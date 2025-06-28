#!/usr/bin/env python3
"""Universal teleoperation entry-point.

Example usages
--------------
$ python3 scripts/start_teleop.py --control keyboard --robot ur5
$ python3 scripts/start_teleop.py --control gamepad  --robot franka
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Ensure ROS workspace Python packages are discoverable when the script is
# launched from the repository root.
ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / "ros_ws" / "src"))

from so101_lerobot.lerobot.factories import ControlFactory, RobotFactory  # noqa: E402


def run():
    parser = argparse.ArgumentParser(description="Universal Teleoperation Interface")
    parser.add_argument("--control", required=True,
                        choices=["vr", "keyboard", "gamepad", "haptic"],
                        help="Control method to use")
    parser.add_argument("--robot", default="so101", help="Robot type to control")
    parser.add_argument("--broadcast-leader", action="store_true",
                        help="Publish commanded pose/gripper to /leader topics so another robot can follow.")
    args = parser.parse_args()

    controller = ControlFactory.create(args.control)
    robot = RobotFactory.create(args.robot)

    if args.broadcast_leader:
        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped
            from std_msgs.msg import Float64
            rclpy.init(args=None)
            node = rclpy.create_node("leader_broadcaster")
            pose_pub = node.create_publisher(PoseStamped, "/leader/pose", 10)
            grip_pub = node.create_publisher(Float64, "/leader/gripper", 10)
        except ImportError:
            print("[Teleop] ROS 2 not available; cannot broadcast leader topics.")
            args.broadcast_leader = False

    print(f"[Teleop] Starting '{args.control}' control for robot '{args.robot}'. Press Ctrl+C to quit.")
    try:
        while True:
            pose = controller.get_pose()
            robot.move_to_pose(pose)
            robot.set_gripper(controller.get_gripper_state())
            if args.broadcast_leader:
                pose_msg = PoseStamped()
                pose_msg.header.stamp = node.get_clock().now().to_msg()
                pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = pose.x, pose.y, pose.z
                pose_pub.publish(pose_msg)
                grip_pub.publish(Float64(data=controller.get_gripper_state()))
                rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        print("\n[Teleop] Shutting down…")


if __name__ == "__main__":
    run() 