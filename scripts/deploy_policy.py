import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseArray
import torch
import lerobot
from lerobot.common.policies.factory import make_policy
import numpy as np
import cv2
from cv_bridge import CvBridge
import argparse
import os

class PolicyDeploymentNode(Node):
    def __init__(self, args):
        super().__init__('policy_deployment_node')
        self.bridge = CvBridge()
        
        # Load the trained LeRobot policy
        self.get_logger().info(f"Loading policy from {args.policy_path}")
        self.policy = make_policy(args.policy_path)
        self.policy.eval()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.policy.to(self.device)
        self.get_logger().info("Policy loaded successfully.")

        # ROS Publishers and Subscribers
        self.action_publisher = self.create_publisher(PoseArray, 'so101_teleop/cmd_pose', 10)
        self.image_subscription = self.create_subscription(Image, '/camera/color/image_raw', self.observation_callback, 10)
        self.joint_state_subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Stored observations
        self.latest_image = None
        self.latest_joint_state = None
        
        # Execution timer
        self.timer = self.create_timer(1.0 / args.frequency, self.execute_policy)

    def observation_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def execute_policy(self):
        if self.latest_image is None or self.latest_joint_state is None:
            self.get_logger().info("Waiting for observations...", throttle_duration_sec=2.0)
            return
            
        # --- PREPARE OBSERVATION FOR POLICY ---
        # This part is highly dependent on how the policy was trained.
        # It needs to match the observation space of the training environment.
        # This is a placeholder for the actual preprocessing required.
        
        # Example: Convert image to tensor
        image_tensor = torch.from_numpy(self.latest_image).permute(2, 0, 1).float().unsqueeze(0).to(self.device)
        
        # Example: Get joint positions
        joint_positions = torch.tensor(self.latest_joint_state.position, dtype=torch.float32).unsqueeze(0).to(self.device)

        # This is a simplified observation dictionary. Adapt as needed.
        observation = {
            'observation.image': image_tensor,
            'observation.state': joint_positions,
        }

        # --- GET ACTION FROM POLICY ---
        with torch.no_grad():
            action = self.policy.select_action(observation)

        # --- PUBLISH ACTION ---
        # This part depends on the action space of your policy.
        # Assuming the policy outputs a flat array for two poses.
        # This is a placeholder for converting the action to a PoseArray.
        
        # Example: Assuming action is a flat array for 2 poses [x,y,z,qx,qy,qz,qw, x,y,z,qx,qy,qz,qw]
        if isinstance(action, np.ndarray):
            action_flat = action.flatten()
        else: # Assuming torch tensor
            action_flat = action.cpu().numpy().flatten()

        if len(action_flat) >= 14: # Check if there is enough data for two poses
            pose_array_msg = PoseArray()
            
            left_pose = geometry_msgs.msg.Pose()
            left_pose.position.x = float(action_flat[0])
            left_pose.position.y = float(action_flat[1])
            left_pose.position.z = float(action_flat[2])
            left_pose.orientation.x = float(action_flat[3])
            left_pose.orientation.y = float(action_flat[4])
            left_pose.orientation.z = float(action_flat[5])
            left_pose.orientation.w = float(action_flat[6])

            right_pose = geometry_msgs.msg.Pose()
            right_pose.position.x = float(action_flat[7])
            right_pose.position.y = float(action_flat[8])
            right_pose.position.z = float(action_flat[9])
            right_pose.orientation.x = float(action_flat[10])
            right_pose.orientation.y = float(action_flat[11])
            right_pose.orientation.z = float(action_flat[12])
            right_pose.orientation.w = float(action_flat[13])

            pose_array_msg.poses = [left_pose, right_pose]
            self.action_publisher.publish(pose_array_msg)
            self.get_logger().info("Published policy action.", throttle_duration_sec=1.0)

def main(args=None):
    parser = argparse.ArgumentParser(description="Deploy a trained LeRobot policy.")
    parser.add_argument(
        "--policy-path",
        type=str,
        required=True,
        help="Path to the trained LeRobot policy directory (e.g., outputs/train/.../checkpoints/00050000).",
    )
    parser.add_argument(
        "--frequency",
        type=float,
        default=10.0,
        help="Frequency (Hz) at which to run the policy.",
    )
    cli_args = parser.parse_args()

    rclpy.init(args=args)
    node = PolicyDeploymentNode(cli_args)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 