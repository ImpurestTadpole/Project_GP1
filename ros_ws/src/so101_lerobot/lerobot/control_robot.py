#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Float64, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
import rosbag2_py
from rosbag2_py import StorageOptions, ConverterOptions
import cv2
from cv_bridge import CvBridge
import numpy as np
import yaml
import os
import time
import argparse
from datetime import datetime
import threading
from typing import Dict, List, Optional, Tuple

# LeRobot imports
try:
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.common.datasets.utils import get_support_camera_ids
    from lerobot.common.policies.utils import get_support_policy_ids
    from lerobot.common.policies.policy import Policy
    from lerobot.common.envs.env import Env
    from lerobot.common.envs.utils import get_support_env_ids
    from lerobot.common.utils.utils import set_seed
    from lerobot.scripts.control_robot import main as lerobot_control_main
    LEROBOT_AVAILABLE = True
except ImportError:
    LEROBOT_AVAILABLE = False
    print("Warning: LeRobot not available. Install with: pip install lerobot")

class ControlRobot(Node):
    def __init__(self, mode='teleop', policy_id=None, env_id=None, dataset_path=None):
        super().__init__('control_robot')
        
        # Mode: 'teleop', 'record', 'replay', 'train', 'eval'
        self.mode = mode
        self.policy_id = policy_id
        self.env_id = env_id
        self.dataset_path = dataset_path
        
        # Load parameters
        self.declare_parameter('robot_type', 'so101_dual_arm')
        self.declare_parameter('use_sim', True)
        self.declare_parameter('update_rate', 100.0)
        self.declare_parameter('safety_timeout', 1.0)
        
        self.robot_type = self.get_parameter('robot_type').value
        self.use_sim = self.get_parameter('use_sim').value
        self.update_rate = self.get_parameter('update_rate').value
        self.safety_timeout = self.get_parameter('safety_timeout').value
        
        # State variables
        self.current_joint_states = None
        self.current_images = {}
        self.current_poses = {}
        self.is_running = False
        self.last_command_time = time.time()
        
        # Publishers
        self.left_arm_pub = self.create_publisher(PoseStamped, '/arm_commands/left', 10)
        self.right_arm_pub = self.create_publisher(PoseStamped, '/arm_commands/right', 10)
        self.left_gripper_pub = self.create_publisher(Float64, '/gripper_commands/left', 10)
        self.right_gripper_pub = self.create_publisher(Float64, '/gripper_commands/right', 10)
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.left_camera_sub = self.create_subscription(
            Image, '/left_camera/image_raw', self.left_camera_callback, 10)
        self.right_camera_sub = self.create_subscription(
            Image, '/right_camera/image_raw', self.right_camera_callback, 10)
        
        # TF
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # CV bridge
        self.bridge = CvBridge()
        
        # LeRobot components
        self.policy = None
        self.env = None
        self.dataset = None
        
        # Recording
        self.recording_writer = None
        self.recording_topics = [
            '/joint_states',
            '/left_camera/image_raw',
            '/right_camera/image_raw',
            '/arm_commands/left',
            '/arm_commands/right',
            '/gripper_commands/left',
            '/gripper_commands/right'
        ]
        
        # Initialize based on mode
        self.initialize_mode()
        
        # Timer for main control loop
        self.control_timer = self.create_timer(1.0/self.update_rate, self.control_loop)
        
        self.get_logger().info(f'Control Robot started in {mode} mode')
        
    def initialize_mode(self):
        """Initialize components based on mode"""
        if self.mode == 'teleop':
            self.initialize_teleop()
        elif self.mode == 'record':
            self.initialize_recording()
        elif self.mode == 'replay':
            self.initialize_replay()
        elif self.mode == 'train':
            self.initialize_training()
        elif self.mode == 'eval':
            self.initialize_evaluation()
        else:
            raise ValueError(f'Unknown mode: {self.mode}')
            
    def initialize_teleop(self):
        """Initialize teleoperation mode"""
        # Subscribe to VR inputs
        self.left_hand_sub = self.create_subscription(
            PoseStamped, '/vr/left_hand', self.left_hand_callback, 10)
        self.right_hand_sub = self.create_subscription(
            PoseStamped, '/vr/right_hand', self.right_hand_callback, 10)
        self.left_gripper_sub = self.create_subscription(
            Float64, '/vr/left_gripper', self.left_gripper_callback, 10)
        self.right_gripper_sub = self.create_subscription(
            Float64, '/vr/right_gripper', self.right_gripper_callback, 10)
            
        self.get_logger().info('Teleoperation mode initialized')
        
    def initialize_recording(self):
        """Initialize recording mode"""
        if not self.dataset_path:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.dataset_path = f'~/data/lerobot_episodes/episode_{timestamp}'
            
        os.makedirs(os.path.expanduser(self.dataset_path), exist_ok=True)
        
        # Initialize rosbag2 writer
        storage_options = StorageOptions(
            uri=os.path.expanduser(self.dataset_path),
            storage_id='sqlite3'
        )
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        
        self.recording_writer = rosbag2_py.SequentialWriter()
        self.recording_writer.open(storage_options, converter_options)
        
        # Add topics
        for topic in self.recording_topics:
            topic_info = rosbag2_py.TopicMetadata(
                name=topic,
                type='unknown',
                serialization_format='cdr'
            )
            self.recording_writer.create_topic(topic_info)
            
        self.get_logger().info(f'Recording mode initialized. Output: {self.dataset_path}')
        
    def initialize_replay(self):
        """Initialize replay mode"""
        if not self.dataset_path:
            raise ValueError("Dataset path required for replay mode")
            
        if not LEROBOT_AVAILABLE:
            raise ImportError("LeRobot required for replay mode")
            
        # Load dataset
        self.dataset = LeRobotDataset(self.dataset_path)
        self.get_logger().info(f'Replay mode initialized. Dataset: {self.dataset_path}')
        
    def initialize_training(self):
        """Initialize training mode"""
        if not LEROBOT_AVAILABLE:
            raise ImportError("LeRobot required for training mode")
            
        if not self.policy_id:
            raise ValueError("Policy ID required for training mode")
            
        # Initialize policy
        self.policy = Policy(self.policy_id)
        
        # Initialize environment
        if self.env_id:
            self.env = Env(self.env_id)
            
        self.get_logger().info(f'Training mode initialized. Policy: {self.policy_id}')
        
    def initialize_evaluation(self):
        """Initialize evaluation mode"""
        if not LEROBOT_AVAILABLE:
            raise ImportError("LeRobot required for evaluation mode")
            
        if not self.policy_id:
            raise ValueError("Policy ID required for evaluation mode")
            
        # Initialize policy
        self.policy = Policy(self.policy_id)
        
        self.get_logger().info(f'Evaluation mode initialized. Policy: {self.policy_id}')
        
    def joint_state_callback(self, msg):
        """Callback for joint states"""
        self.current_joint_states = msg
        
    def left_camera_callback(self, msg):
        """Callback for left camera"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_images['left_camera'] = cv_image
        except Exception as e:
            self.get_logger().error(f'Failed to convert left camera image: {e}')
            
    def right_camera_callback(self, msg):
        """Callback for right camera"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_images['right_camera'] = cv_image
        except Exception as e:
            self.get_logger().error(f'Failed to convert right camera image: {e}')
            
    def left_hand_callback(self, msg):
        """Callback for left hand pose"""
        self.current_poses['left_hand'] = msg
        self.last_command_time = time.time()
        
    def right_hand_callback(self, msg):
        """Callback for right hand pose"""
        self.current_poses['right_hand'] = msg
        self.last_command_time = time.time()
        
    def left_gripper_callback(self, msg):
        """Callback for left gripper"""
        self.current_poses['left_gripper'] = msg
        self.last_command_time = time.time()
        
    def right_gripper_callback(self, msg):
        """Callback for right gripper"""
        self.current_poses['right_gripper'] = msg
        self.last_command_time = time.time()
        
    def control_loop(self):
        """Main control loop"""
        current_time = time.time()
        
        # Safety check
        if current_time - self.last_command_time > self.safety_timeout:
            self.get_logger().warn('No commands received. Stopping robot.')
            return
            
        # Mode-specific control
        if self.mode == 'teleop':
            self.teleop_control()
        elif self.mode == 'record':
            self.record_control()
        elif self.mode == 'replay':
            self.replay_control()
        elif self.mode == 'train':
            self.train_control()
        elif self.mode == 'eval':
            self.eval_control()
            
    def teleop_control(self):
        """Teleoperation control"""
        # Publish arm commands
        if 'left_hand' in self.current_poses:
            self.left_arm_pub.publish(self.current_poses['left_hand'])
            
        if 'right_hand' in self.current_poses:
            self.right_arm_pub.publish(self.current_poses['right_hand'])
            
        # Publish gripper commands
        if 'left_gripper' in self.current_poses:
            self.left_gripper_pub.publish(self.current_poses['left_gripper'])
            
        if 'right_gripper' in self.current_poses:
            self.right_gripper_pub.publish(self.current_poses['right_gripper'])
            
        # Record if recording is enabled
        if self.recording_writer:
            self.record_data()
            
    def record_control(self):
        """Recording control"""
        # Record current state
        self.record_data()
        
    def replay_control(self):
        """Replay control"""
        if not self.dataset:
            return
            
        # Get next action from dataset
        # This is a simplified implementation
        # In practice, you'd iterate through the dataset
        pass
        
    def train_control(self):
        """Training control"""
        if not self.policy:
            return
            
        # Get current observation
        observation = self.get_observation()
        
        # Get action from policy
        action = self.policy.predict(observation)
        
        # Execute action
        self.execute_action(action)
        
    def eval_control(self):
        """Evaluation control"""
        if not self.policy:
            return
            
        # Similar to training but for evaluation
        observation = self.get_observation()
        action = self.policy.predict(observation)
        self.execute_action(action)
        
    def get_observation(self):
        """Get current observation for policy"""
        observation = {}
        
        # Joint states
        if self.current_joint_states:
            observation['joint_states'] = self.current_joint_states
            
        # Images
        for camera_id, image in self.current_images.items():
            observation[camera_id] = image
            
        # Poses
        for pose_id, pose in self.current_poses.items():
            observation[pose_id] = pose
            
        return observation
        
    def execute_action(self, action):
        """Execute action from policy"""
        # This is a simplified implementation
        # In practice, you'd parse the action and send appropriate commands
        
        if 'left_arm' in action:
            self.left_arm_pub.publish(action['left_arm'])
            
        if 'right_arm' in action:
            self.right_arm_pub.publish(action['right_arm'])
            
        if 'left_gripper' in action:
            self.left_gripper_pub.publish(action['left_gripper'])
            
        if 'right_gripper' in action:
            self.right_gripper_pub.publish(action['right_gripper'])
            
    def record_data(self):
        """Record current data to rosbag"""
        if not self.recording_writer:
            return
            
        try:
            # Record joint states
            if self.current_joint_states:
                self.recording_writer.write('/joint_states', self.current_joint_states, 
                                         self.get_clock().now().nanoseconds)
                
            # Record images
            for camera_id, image in self.current_images.items():
                topic = f'/{camera_id}/image_raw'
                ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
                self.recording_writer.write(topic, ros_image, 
                                         self.get_clock().now().nanoseconds)
                
            # Record poses
            for pose_id, pose in self.current_poses.items():
                if 'hand' in pose_id:
                    topic = f'/arm_commands/{pose_id.split("_")[0]}'
                    self.recording_writer.write(topic, pose, 
                                             self.get_clock().now().nanoseconds)
                elif 'gripper' in pose_id:
                    topic = f'/gripper_commands/{pose_id.split("_")[0]}'
                    self.recording_writer.write(topic, pose, 
                                             self.get_clock().now().nanoseconds)
                    
        except Exception as e:
            self.get_logger().error(f'Failed to record data: {e}')
            
    def cleanup(self):
        """Cleanup resources"""
        if self.recording_writer:
            self.recording_writer.close()

def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Control SO-101 robot with LeRobot')
    parser.add_argument('--mode', choices=['teleop', 'record', 'replay', 'train', 'eval'], 
                       default='teleop', help='Control mode')
    parser.add_argument('--policy-id', help='Policy ID for training/evaluation')
    parser.add_argument('--env-id', help='Environment ID for training')
    parser.add_argument('--dataset-path', help='Dataset path for replay/recording')
    
    args, unknown = parser.parse_known_args()
    
    node = ControlRobot(
        mode=args.mode,
        policy_id=args.policy_id,
        env_id=args.env_id,
        dataset_path=args.dataset_path
    )
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 