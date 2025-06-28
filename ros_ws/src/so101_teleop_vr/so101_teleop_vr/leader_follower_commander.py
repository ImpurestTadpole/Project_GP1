#!/usr/bin/env python3
"""Leader-Follower Commander

A simple node that mirrors the leader robot's end-effector pose (or joint
states) onto a follower robot.  Intended for teleoperation setups where a
hand-held or separate robot (leader) drives the motions of the primary
robot (follower).

This is only a minimal reference implementation – adapt the topic names and
IK handling to suit your hardware.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header

class LeaderFollowerCommander(Node):
    def __init__(self):
        super().__init__('leader_follower_commander')

        # Declare parameters
        self.declare_parameter('mode', 'joint')  # 'joint' or 'cartesian'
        self.declare_parameter('update_rate', 100.0)
        self.declare_parameter('leader_namespace', '/leader')
        self.declare_parameter('follower_namespace', '/follower')

        self.mode = self.get_parameter('mode').value
        self.rate = float(self.get_parameter('update_rate').value)
        self.leader_ns = self.get_parameter('leader_namespace').value.rstrip('/')
        self.follower_ns = self.get_parameter('follower_namespace').value.rstrip('/')

        # Publishers / Subscribers
        if self.mode == 'joint':
            self.leader_joint_sub = self.create_subscription(
                JointState,
                f'{self.leader_ns}/joint_states',
                self.joint_state_callback,
                10,
            )
            self.follower_joint_pub = self.create_publisher(
                JointTrajectory,
                f'{self.follower_ns}/joint_trajectory_cmd',
                10,
            )
        else:
            # Cartesian mirroring
            self.leader_pose_sub = self.create_subscription(
                PoseStamped,
                f'{self.leader_ns}/ee_pose',
                self.pose_callback,
                10,
            )
            self.follower_pose_pub = self.create_publisher(
                PoseStamped,
                f'{self.follower_ns}/command_pose',
                10,
            )

        self.get_logger().info(
            f'Leader-Follower commander started in {self.mode} mode –  '  # noqa: E501
            f'leader ns: {self.leader_ns}, follower ns: {self.follower_ns}')

    # ----- JOINT MODE ----------------------------------------------------
    def joint_state_callback(self, msg: JointState):
        traj = JointTrajectory()
        traj.header = Header()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = msg.name
        point = JointTrajectoryPoint()
        point.positions = msg.position
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(1e9 / self.rate)
        traj.points.append(point)
        self.follower_joint_pub.publish(traj)

    # ----- CARTESIAN MODE -------------------------------------------------
    def pose_callback(self, msg: PoseStamped):
        self.follower_pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderFollowerCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 