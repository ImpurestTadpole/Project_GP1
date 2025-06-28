from abc import ABC, abstractmethod
from typing import Any

class RobotInterface(ABC):
    """Abstract base class that exposes a minimal Cartesian control API.

    A concrete implementation acts as an adapter between the high-level control
    logic and the low-level robot driver (ROS 2 control, custom drivers, etc.).
    """

    @abstractmethod
    def move_to_pose(self, pose: "Pose6D", speed: float = 1.0) -> None:
        """Send the robot an end-effector pose target.

        Parameters
        ----------
        pose : Pose6D
            Target pose expressed in the robot base frame.
        speed : float, optional
            Scaling factor in \[0, 1] for limiting trajectory speed. Default is 1.
        """
        raise NotImplementedError

    @abstractmethod
    def get_current_pose(self) -> "Pose6D":
        """Return the measured end-effector pose."""
        raise NotImplementedError

    @abstractmethod
    def set_gripper(self, state: float) -> None:
        """Command the gripper opening fraction in \[0.0, 1.0]."""
        raise NotImplementedError

    # Optional convenience helpers ------------------------------------------------

    def stop_motion(self) -> None:
        """Stop all motion immediately (override in subclasses if available)."""
        pass

    def execute_trajectory(self, poses: list["Pose6D"], speed: float = 1.0) -> None:
        """Execute a list of Cartesian waypoints (blocking).

        Default implementation falls back to successive *move_to_pose* calls if
        the subclass does not override a more efficient method.
        """
        for p in poses:
            self.move_to_pose(p, speed) 