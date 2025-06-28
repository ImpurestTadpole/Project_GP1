from __future__ import annotations

from typing import Dict, Any

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Float64
except ImportError:
    rclpy = None  # type: ignore

from ..control_interface import ControlInterface
from .keyboard_control import Pose6D


class LeaderFollowerControlInterface(ControlInterface):
    """Follower side of leader-follower teleoperation.

    Listens to `/leader/pose` and `/leader/gripper` topics and mirrors them.
    If ROS 2 is not available this interface raises an error.
    """

    def __init__(self, pose_topic: str = "/leader/pose", gripper_topic: str = "/leader/gripper") -> None:
        if rclpy is None:
            raise RuntimeError("Leader-Follower control requires ROS 2 runtime.")

        rclpy.init(args=None)
        self._node = rclpy.create_node("leader_follower_control")

        self._pose_topic = pose_topic
        self._gripper_topic = gripper_topic

        self._pose = Pose6D.identity()
        self._gripper = 0.0

        self._node.create_subscription(PoseStamped, pose_topic, self._pose_cb, 10)
        self._node.create_subscription(Float64, gripper_topic, self._gripper_cb, 10)

    # ------------------------------------------------------------------
    # ROS Callbacks -----------------------------------------------------
    # ------------------------------------------------------------------

    def _pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        self._pose.x, self._pose.y, self._pose.z = p.x, p.y, p.z
        # Orientation ignored in this stub

    def _gripper_cb(self, msg: Float64):
        self._gripper = float(msg.data)

    # ------------------------------------------------------------------
    # ControlInterface API ---------------------------------------------
    # ------------------------------------------------------------------

    def get_pose(self) -> Pose6D:
        rclpy.spin_once(self._node, timeout_sec=0.0)
        return self._pose

    def get_gripper_state(self) -> float:
        rclpy.spin_once(self._node, timeout_sec=0.0)
        return self._gripper

    def get_user_commands(self) -> Dict[str, Any]:
        return {}

    def provide_feedback(self, feedback: Any):
        pass

    # ------------------------------------------------------------------
    def __del__(self):
        try:
            rclpy.shutdown()
        except Exception:
            pass 