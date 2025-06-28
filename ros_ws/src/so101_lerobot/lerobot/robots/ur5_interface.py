from __future__ import annotations

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Float64
except ImportError:
    rclpy = None  # type: ignore

from ..robot_interface import RobotInterface
from ..control.keyboard_control import Pose6D


class UR5Interface(RobotInterface):
    """Minimal stub for UR5 Cartesian control.

    The implementation only logs the requested pose. A production
    implementation should interface with *ur_robot_driver* or RTDE.
    """

    def __init__(self):
        if rclpy is not None:
            rclpy.init(args=None)
            self._node = rclpy.create_node("ur5_interface")
            # Publishers would be created here
        else:
            self._node = None

        self._current_pose = Pose6D.identity()

    def move_to_pose(self, pose: Pose6D, speed: float = 1.0):
        # For demonstration we just store the pose.
        self._current_pose = pose
        if self._node:
            self._node.get_logger().info(f"UR5 target pose: {pose}")
        else:
            print(f"[UR5Interface] Target pose: {pose}")

    def get_current_pose(self) -> Pose6D:
        return self._current_pose

    def set_gripper(self, state: float):
        print(f"[UR5Interface] Gripper state set to {state:.2f}") 