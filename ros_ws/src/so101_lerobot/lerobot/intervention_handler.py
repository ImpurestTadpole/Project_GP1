#!/usr/bin/env python3
"""
Intervention Handler for HIL-SERL
Manages human interventions during RL policy execution
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool, Float32
import numpy as np
import time
from collections import deque


class InterventionHandler(Node):
    """Handles human interventions during RL training"""
    
    def __init__(self):
        super().__init__('intervention_handler')
        
        # Intervention state
        self.intervention_active = False
        self.intervention_start_time = None
        self.intervention_trajectory = deque(maxlen=1000)
        self.baseline_trajectory = deque(maxlen=1000)
        
        # VR controller mapping
        self.controller_deadzone = 0.1
        self.intervention_button = 0  # A button on Quest controller
        self.preference_buttons = [1, 2]  # B and X buttons for good/bad feedback
        
        # ROS interface
        self.setup_ros_interface()
        
        self.get_logger().info("Intervention Handler initialized")

    def setup_ros_interface(self):
        """Set up ROS publishers and subscribers"""
        # Subscribers
        self.vr_joy_sub = self.create_subscription(
            Joy, '/vr/controller/joy',
            self.vr_controller_callback, 10
        )
        self.policy_action_sub = self.create_subscription(
            PoseArray, '/rl_policy/action',
            self.policy_action_callback, 10
        )
        
        # Publishers
        self.intervention_pub = self.create_publisher(
            Bool, '/hil_serl/intervention', 10
        )
        self.corrected_action_pub = self.create_publisher(
            PoseArray, '/so101_teleop/cmd_pose', 10
        )
        self.preference_pub = self.create_publisher(
            Float32, '/hil_serl/preference', 10
        )
        self.intervention_data_pub = self.create_publisher(
            PoseArray, '/hil_serl/intervention_trajectory', 10
        )
        
        # Timer for intervention processing
        self.timer = self.create_timer(0.02, self.process_intervention)  # 50Hz

    def vr_controller_callback(self, msg):
        """Process VR controller input for interventions"""
        if len(msg.buttons) <= max(self.intervention_button, max(self.preference_buttons)):
            return
            
        # Check intervention button
        intervention_pressed = msg.buttons[self.intervention_button]
        
        if intervention_pressed and not self.intervention_active:
            self.start_intervention()
        elif not intervention_pressed and self.intervention_active:
            self.end_intervention()
            
        # Check preference buttons
        if msg.buttons[self.preference_buttons[0]]:  # Good feedback
            self.publish_preference(1.0)
        elif msg.buttons[self.preference_buttons[1]]:  # Bad feedback  
            self.publish_preference(0.0)
            
        # Store controller pose for intervention
        if self.intervention_active:
            self.process_vr_input(msg)

    def policy_action_callback(self, msg):
        """Store baseline policy actions for comparison"""
        if not self.intervention_active:
            self.baseline_trajectory.append({
                'timestamp': time.time(),
                'poses': msg.poses,
                'type': 'policy'
            })

    def start_intervention(self):
        """Start human intervention"""
        self.intervention_active = True
        self.intervention_start_time = time.time()
        self.intervention_trajectory.clear()
        
        # Publish intervention status
        msg = Bool()
        msg.data = True
        self.intervention_pub.publish(msg)
        
        self.get_logger().info("Human intervention started")

    def end_intervention(self):
        """End human intervention and store data"""
        self.intervention_active = False
        intervention_duration = time.time() - self.intervention_start_time
        
        # Publish intervention status
        msg = Bool()
        msg.data = False
        self.intervention_pub.publish(msg)
        
        # Store intervention data for learning
        self.store_intervention_data()
        
        self.get_logger().info(f"Human intervention ended (duration: {intervention_duration:.2f}s)")

    def process_vr_input(self, joy_msg):
        """Convert VR controller input to robot poses during intervention"""
        if len(joy_msg.axes) < 6:
            return
            
        # Extract controller pose from joy message
        # This assumes the VR system publishes controller poses in joy axes
        left_pose = Pose()
        right_pose = Pose()
        
        # Map VR controller positions to robot poses
        # This is a simplified mapping - you'd need proper calibration
        left_pose.position.x = joy_msg.axes[0] * 0.5  # Scale factor
        left_pose.position.y = joy_msg.axes[1] * 0.5
        left_pose.position.z = joy_msg.axes[2] * 0.5 + 0.3  # Offset
        
        right_pose.position.x = joy_msg.axes[3] * 0.5
        right_pose.position.y = joy_msg.axes[4] * 0.5  
        right_pose.position.z = joy_msg.axes[5] * 0.5 + 0.3
        
        # Store intervention trajectory
        self.intervention_trajectory.append({
            'timestamp': time.time(),
            'left_pose': left_pose,
            'right_pose': right_pose,
            'type': 'intervention'
        })

    def process_intervention(self):
        """Process and publish intervention commands"""
        if not self.intervention_active or len(self.intervention_trajectory) == 0:
            return
            
        # Get latest intervention command
        latest_command = self.intervention_trajectory[-1]
        
        # Create pose array message
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "base_link"
        pose_array.poses = [latest_command['left_pose'], latest_command['right_pose']]
        
        # Publish corrected action
        self.corrected_action_pub.publish(pose_array)

    def store_intervention_data(self):
        """Store intervention data for reward learning"""
        if len(self.intervention_trajectory) == 0:
            return
            
        # Create intervention trajectory message
        trajectory_msg = PoseArray()
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        trajectory_msg.header.frame_id = "base_link"
        
        # Convert intervention trajectory to pose array
        poses = []
        for point in self.intervention_trajectory:
            poses.extend([point['left_pose'], point['right_pose']])
        
        trajectory_msg.poses = poses
        self.intervention_data_pub.publish(trajectory_msg)
        
        self.get_logger().info(f"Stored intervention trajectory with {len(self.intervention_trajectory)} points")

    def publish_preference(self, score):
        """Publish human preference score"""
        msg = Float32()
        msg.data = score
        self.preference_pub.publish(msg)
        
        feedback_type = "positive" if score > 0.5 else "negative"
        self.get_logger().info(f"Published {feedback_type} preference feedback")

    def get_intervention_statistics(self):
        """Get statistics about interventions"""
        return {
            'total_interventions': len(self.intervention_trajectory),
            'average_duration': np.mean([
                point['timestamp'] - self.intervention_start_time 
                for point in self.intervention_trajectory
            ]) if self.intervention_trajectory else 0,
            'intervention_rate': len(self.intervention_trajectory) / max(time.time() - self.intervention_start_time, 1)
            if self.intervention_start_time else 0
        }


def main(args=None):
    rclpy.init(args=args)
    
    handler = InterventionHandler()
    
    try:
        rclpy.spin(handler)
    except KeyboardInterrupt:
        handler.get_logger().info("Intervention handler stopped")
    finally:
        handler.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 