from abc import ABC, abstractmethod
from typing import Any, Dict

class ControlInterface(ABC):
    """Abstract base class for all teleoperation input devices.

    Concrete subclasses must implement the low-level logic for reading the
    operator's inputs and converting them into 6-DoF end-effector targets as
    well as gripper commands and any auxiliary user interface commands.
    """

    @abstractmethod
    def get_pose(self) -> "Pose6D":
        """Return the desired end-effector pose in the robot base frame."""
        raise NotImplementedError

    @abstractmethod
    def get_gripper_state(self) -> float:
        """Return the desired gripper opening fraction in \[0.0, 1.0]."""
        raise NotImplementedError

    @abstractmethod
    def get_user_commands(self) -> Dict[str, Any]:
        """Return a dictionary of high-level user commands (e.g. buttons)."""
        raise NotImplementedError

    @abstractmethod
    def provide_feedback(self, feedback: Any) -> None:
        """Optionally send haptic/visual/auditory feedback back to the user."""
        raise NotImplementedError 