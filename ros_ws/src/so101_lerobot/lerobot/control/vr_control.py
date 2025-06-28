from __future__ import annotations

from typing import Dict, Any

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Float64
except ImportError:  # ROS not available in pure python env
    rclpy = None  # type: ignore

from ..control_interface import ControlInterface
from .keyboard_control import Pose6D


class VRControlInterface(ControlInterface):
    """ROS2-based interface for Meta Quest or similar streaming VR inputs.

    Subscribes to /vr/left_hand, /vr/right_hand, etc. if ROS 2 is available.
    If ROS is not available (e.g. during unit testing) the class falls back to
    returning the identity pose.
    """

    def __init__(self):
        self._pose = Pose6D.identity()
        self._gripper = 0.0

        if rclpy is not None:
            rclpy.init(args=None)
            self._node = rclpy.create_node("vr_control_interface")
            self._subscription = self._node.create_subscription(
                PoseStamped, "/vr/right_hand", self._pose_cb, 10
            )
            self._gripper_sub = self._node.create_subscription(
                Float64, "/vr/right_gripper", self._gripper_cb, 10
            )
        else:
            self._node = None

    # ------------------------------------------------------------------
    # ROS Callbacks -----------------------------------------------------
    # ------------------------------------------------------------------

    def _pose_cb(self, msg):
        p = msg.pose.position
        self._pose.x, self._pose.y, self._pose.z = p.x, p.y, p.z
        # Orientation omitted for brevity

    def _gripper_cb(self, msg):
        self._gripper = float(msg.data)

    # ------------------------------------------------------------------
    # ControlInterface API ---------------------------------------------
    # ------------------------------------------------------------------

    def get_pose(self) -> Pose6D:
        if self._node is not None:
            rclpy.spin_once(self._node, timeout_sec=0.0)
        return self._pose

    def get_gripper_state(self) -> float:
        if self._node is not None:
            rclpy.spin_once(self._node, timeout_sec=0.0)
        return self._gripper

    def get_user_commands(self) -> Dict[str, Any]:
        return {}

    def provide_feedback(self, feedback: Any):
        pass

    # ------------------------------------------------------------------
    def __del__(self):
        if self._node is not None:
            rclpy.shutdown() 