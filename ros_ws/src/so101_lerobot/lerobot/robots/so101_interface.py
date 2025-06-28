from __future__ import annotations

from ..robot_interface import RobotInterface
from ..control.keyboard_control import Pose6D


class SO101Interface(RobotInterface):
    """Stub for existing SO-101 dual arm robot. Delegates to existing control code if desired."""

    def __init__(self):
        self._current_pose = Pose6D.identity()

    def move_to_pose(self, pose: Pose6D, speed: float = 1.0):
        self._current_pose = pose
        print(f"[SO101Interface] Target pose: {pose}")

    def get_current_pose(self) -> Pose6D:
        return self._current_pose

    def set_gripper(self, state: float):
        print(f"[SO101Interface] Gripper state set to {state:.2f}") 