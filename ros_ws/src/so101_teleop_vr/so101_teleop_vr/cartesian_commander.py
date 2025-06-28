#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
import numpy as np
import yaml
import os
import time
from datetime import datetime

class CartesianCommander(Node):
    def __init__(self):
        super().__init__('cartesian_commander')
        
        # Load parameters
        self.declare_parameter('vr_params_file', '~/config/vr_params.yaml')
        self.declare_parameter('mode', 'sim')  # 'sim' or 'real'
        self.declare_parameter('update_rate', 100.0)
        
        vr_params_file = self.get_parameter('vr_params_file').value
        self.mode = self.get_parameter('mode').value
        self.update_rate = self.get_parameter('update_rate').value
        
        self.vr_params = self.load_vr_params(vr_params_file)
        
        # Publishers
        self.left_arm_pub = self.create_publisher(PoseStamped, '/arm_commands/left', 10)
        self.right_arm_pub = self.create_publisher(PoseStamped, '/arm_commands/right', 10)
        self.left_gripper_pub = self.create_publisher(Float64, '/gripper_commands/left', 10)
        self.right_gripper_pub = self.create_publisher(Float64, '/gripper_commands/right', 10)
        
        # Joint trajectory publishers for real robot
        self.left_arm_traj_pub = self.create_publisher(JointTrajectory, '/left_arm_controller/joint_trajectory', 10)
        self.right_arm_traj_pub = self.create_publisher(JointTrajectory, '/right_arm_controller/joint_trajectory', 10)
        
        # Subscribers
        self.left_hand_sub = self.create_subscription(
            PoseStamped, '/vr/left_hand', self.left_hand_callback, 10)
        self.right_hand_sub = self.create_subscription(
            PoseStamped, '/vr/right_hand', self.right_hand_callback, 10)
        self.left_gripper_sub = self.create_subscription(
            Float64, '/vr/left_gripper', self.left_gripper_callback, 10)
        self.right_gripper_sub = self.create_subscription(
            Float64, '/vr/right_gripper', self.right_gripper_callback, 10)
        
        # Joint state subscriber for IK
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # TF
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # State variables
        self.left_hand_pose = None
        self.right_hand_pose = None
        self.left_gripper_value = 0.0
        self.right_gripper_value = 0.0
        self.current_joint_states = None
        
        # Timer for publishing commands
        self.timer = self.create_timer(1.0/self.update_rate, self.publish_commands)
        
        # Safety variables
        self.last_command_time = time.time()
        self.command_timeout = 1.0  # seconds
        
        self.get_logger().info(f'Cartesian Commander started in {self.mode} mode')
        
    def load_vr_params(self, params_file):
        """Load VR parameters from YAML file"""
        try:
            with open(os.path.expanduser(params_file), 'r') as f:
                return yaml.safe_load(f)
        except FileNotFoundError:
            self.get_logger().warn(f'VR params file not found: {params_file}. Using defaults.')
            return self.get_default_params()
            
    def get_default_params(self):
        """Get default VR parameters"""
        return {
            'vr_teleoperation': {
                'pose_scaling': {
                    'position': {'scale': [1.0, 1.0, 1.0], 'offset': [0.0, 0.0, 0.0]},
                    'orientation': {'scale': [1.0, 1.0, 1.0], 'offset': [0.0, 0.0, 0.0]}
                },
                'workspace_limits': {
                    'position': {'x': [-0.8, 0.8], 'y': [-0.6, 0.6], 'z': [0.2, 1.2]}
                },
                'safety': {'max_velocity': 0.5}
            }
        }
        
    def left_hand_callback(self, msg):
        """Callback for left hand pose"""
        self.left_hand_pose = self.process_pose(msg)
        self.last_command_time = time.time()
        
    def right_hand_callback(self, msg):
        """Callback for right hand pose"""
        self.right_hand_pose = self.process_pose(msg)
        self.last_command_time = time.time()
        
    def left_gripper_callback(self, msg):
        """Callback for left gripper value"""
        self.left_gripper_value = msg.data
        self.last_command_time = time.time()
        
    def right_gripper_callback(self, msg):
        """Callback for right gripper value"""
        self.right_gripper_value = msg.data
        self.last_command_time = time.time()
        
    def joint_state_callback(self, msg):
        """Callback for joint states"""
        self.current_joint_states = msg
        
    def process_pose(self, pose_msg):
        """Process and validate pose"""
        pose = PoseStamped()
        pose.header = pose_msg.header
        
        # Apply scaling and offset
        scale = self.vr_params['vr_teleoperation']['pose_scaling']['position']['scale']
        offset = self.vr_params['vr_teleoperation']['pose_scaling']['position']['offset']
        
        pose.pose.position.x = pose_msg.pose.position.x * scale[0] + offset[0]
        pose.pose.position.y = pose_msg.pose.position.y * scale[1] + offset[1]
        pose.pose.position.z = pose_msg.pose.position.z * scale[2] + offset[2]
        
        pose.pose.orientation = pose_msg.pose.orientation
        
        # Apply workspace limits
        limits = self.vr_params['vr_teleoperation']['workspace_limits']['position']
        pose.pose.position.x = np.clip(pose.pose.position.x, limits['x'][0], limits['x'][1])
        pose.pose.position.y = np.clip(pose.pose.position.y, limits['y'][0], limits['y'][1])
        pose.pose.position.z = np.clip(pose.pose.position.z, limits['z'][0], limits['z'][1])
        
        return pose
        
    def publish_commands(self):
        """Publish arm and gripper commands"""
        current_time = time.time()
        
        # Check for command timeout
        if current_time - self.last_command_time > self.command_timeout:
            self.get_logger().warn('No VR commands received for too long. Stopping robot.')
            return
        
        # Publish arm commands
        if self.left_hand_pose:
            if self.mode == 'sim':
                self.left_arm_pub.publish(self.left_hand_pose)
            else:
                self.publish_joint_trajectory('left', self.left_hand_pose)
            
        if self.right_hand_pose:
            if self.mode == 'sim':
                self.right_arm_pub.publish(self.right_hand_pose)
            else:
                self.publish_joint_trajectory('right', self.right_hand_pose)
            
        # Publish gripper commands
        left_gripper_msg = Float64()
        left_gripper_msg.data = self.left_gripper_value
        self.left_gripper_pub.publish(left_gripper_msg)
        
        right_gripper_msg = Float64()
        right_gripper_msg.data = self.right_gripper_value
        self.right_gripper_pub.publish(right_gripper_msg)
        
        # Publish TF for visualization
        self.publish_tf()
        
    def publish_joint_trajectory(self, arm, pose):
        """Publish joint trajectory for real robot (simplified IK)"""
        # This is a simplified IK - in practice, you'd use MoveIt or a proper IK solver
        if self.current_joint_states is None:
            return
            
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        
        if arm == 'left':
            trajectory.joint_names = ['left_shoulder_pitch', 'left_shoulder_roll', 'left_elbow_pitch', 'left_wrist_pitch']
            publisher = self.left_arm_traj_pub
        else:
            trajectory.joint_names = ['right_shoulder_pitch', 'right_shoulder_roll', 'right_elbow_pitch', 'right_wrist_pitch']
            publisher = self.right_arm_traj_pub
            
        # Simple inverse kinematics (placeholder)
        # In practice, use MoveIt or a proper IK solver
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0, 0.0]  # Placeholder
        point.velocities = [0.0, 0.0, 0.0, 0.0]
        point.accelerations = [0.0, 0.0, 0.0, 0.0]
        point.time_from_start.sec = 1
        
        trajectory.points.append(point)
        publisher.publish(trajectory)
        
    def publish_tf(self):
        """Publish TF transforms for visualization"""
        if self.left_hand_pose:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'vr_left_hand'
            t.transform.translation.x = self.left_hand_pose.pose.position.x
            t.transform.translation.y = self.left_hand_pose.pose.position.y
            t.transform.translation.z = self.left_hand_pose.pose.position.z
            t.transform.rotation = self.left_hand_pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)
            
        if self.right_hand_pose:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'vr_right_hand'
            t.transform.translation.x = self.right_hand_pose.pose.position.x
            t.transform.translation.y = self.right_hand_pose.pose.position.y
            t.transform.translation.z = self.right_hand_pose.pose.position.z
            t.transform.rotation = self.right_hand_pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = CartesianCommander()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 