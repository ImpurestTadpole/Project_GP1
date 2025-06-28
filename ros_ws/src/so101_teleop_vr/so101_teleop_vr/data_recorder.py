#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState, CameraInfo
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import rerun as rr
import os
import json
from datetime import datetime

class DataRecorderNode(Node):
    def __init__(self):
        super().__init__('data_recorder_node')
        self.bridge = CvBridge()
        
        # Rerun initialization
        rr.init("so101_teleop_data", spawn=True)

        # Base directory for raw episode data
        self.output_dir = os.path.expanduser('~/data/raw_episodes')
        os.makedirs(self.output_dir, exist_ok=True)
        
        # ROS Subscriptions
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(PoseArray, 'so101_teleop/cmd_pose', self.pose_callback, 10)
        self.create_subscription(String, 'so101_teleop/rec_control', self.control_callback, 10)

        self.recording = False
        self.episode_path = ""
        self.joint_states_file = None
        self.poses_file = None
        self.frame_count = 0
        self.get_logger().info('Data Recorder Node started. Waiting for command...')

        self.declare_parameter('camera_type', 'usb')
        self.camera_type = self.get_parameter('camera_type').value

        # Subscriptions for RGB and optional Depth
        self.rgb_subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.rgb_callback, 10)
        
        if self.camera_type == 'realsense':
            self.depth_subscription = self.create_subscription(
                Image, '/camera/depth/image_rect_raw', self.depth_callback, 10)
            self.get_logger().info("RealSense camera selected, subscribing to depth topic.")
        
        self.latest_rgb_image = None
        self.latest_depth_image = None

    def image_callback(self, msg):
        if self.recording:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                # Log to Rerun
                rr.log(f"robot/camera/image", rr.Image(cv_image))
                # Save image to file
                image_filename = os.path.join(self.episode_path, "images", f"frame_{self.frame_count:05d}.png")
                cv2.imwrite(image_filename, cv_image)
                self.frame_count += 1
            except Exception as e:
                self.get_logger().error(f"Error processing image: {e}")

    def joint_state_callback(self, msg):
        if self.recording:
            # Log to Rerun
            rr.log("robot/joint_states/positions", rr.TimeSeriesScalar(msg.position, names=msg.name))
            # Save to JSONL file
            data = {
                'timestamp': self.get_clock().now().nanoseconds,
                'name': list(msg.name),
                'position': list(msg.position),
                'velocity': list(msg.velocity),
                'effort': list(msg.effort),
            }
            self.joint_states_file.write(json.dumps(data) + '\n')

    def pose_callback(self, msg):
        if self.recording and len(msg.poses) >= 2:
            # Log to Rerun
            left_pos = msg.poses[0].position
            left_orient = msg.poses[0].orientation
            rr.log("vr/controller/left", rr.Transform3D(
                translation=[left_pos.x, left_pos.y, left_pos.z],
                rotation=rr.Quaternion(xyzw=[left_orient.x, left_orient.y, left_orient.z, left_orient.w])
            ))
            right_pos = msg.poses[1].position
            right_orient = msg.poses[1].orientation
            rr.log("vr/controller/right", rr.Transform3D(
                translation=[right_pos.x, right_pos.y, right_pos.z],
                rotation=rr.Quaternion(xyzw=[right_orient.x, right_orient.y, right_orient.z, right_orient.w])
            ))
            # Save to JSONL file
            data = {
                'timestamp': self.get_clock().now().nanoseconds,
                'left_pose': {'position': [left_pos.x, left_pos.y, left_pos.z], 'orientation': [left_orient.x, left_orient.y, left_orient.z, left_orient.w]},
                'right_pose': {'position': [right_pos.x, right_pos.y, right_pos.z], 'orientation': [right_orient.x, right_orient.y, right_orient.z, right_orient.w]},
            }
            self.poses_file.write(json.dumps(data) + '\n')

    def control_callback(self, msg):
        if msg.data == 'start' and not self.recording:
            self.recording = True
            self.frame_count = 0
            # Create a new directory for this episode
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.episode_path = os.path.join(self.output_dir, f'episode_{timestamp}')
            os.makedirs(os.path.join(self.episode_path, 'images'), exist_ok=True)

            # Open files for writing
            self.joint_states_file = open(os.path.join(self.episode_path, 'joint_states.jsonl'), 'w')
            self.poses_file = open(os.path.join(self.episode_path, 'controller_poses.jsonl'), 'w')

            self.get_logger().info(f'Started recording to {self.episode_path}')
            rr.log("timeline", rr.TextLog(f"Recording started: {self.episode_path}", level=rr.TextLogLevel.INFO))
        
        elif msg.data == 'stop' and self.recording:
            self.recording = False
            # Close files
            if self.joint_states_file: self.joint_states_file.close()
            if self.poses_file: self.poses_file.close()
            
            self.get_logger().info(f'Stopped recording episode. Data saved in {self.episode_path}')
            rr.log("timeline", rr.TextLog("Recording stopped", level=rr.TextLogLevel.INFO))
            self.episode_path = ""

    def rgb_callback(self, msg):
        self.latest_rgb_image = msg

    def depth_callback(self, msg):
        self.latest_depth_image = msg

    def save_data(self):
        if self.latest_rgb_image:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(self.latest_rgb_image, "bgr8")
                cv2.imwrite(str(self.episode_path / f"frame_{self.frame_count:04d}.png"), cv_image)
            except Exception as e:
                self.get_logger().error(f"Failed to save RGB image: {e}")

        if self.camera_type == 'realsense' and self.latest_depth_image:
            try:
                # Depth images are typically 16UC1, save as is.
                depth_image = self.bridge.imgmsg_to_cv2(self.latest_depth_image, desired_encoding='passthrough')
                cv2.imwrite(str(self.episode_path / f"depth_{self.frame_count:04d}.png"), depth_image)
            except Exception as e:
                self.get_logger().error(f"Failed to save depth image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DataRecorderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 