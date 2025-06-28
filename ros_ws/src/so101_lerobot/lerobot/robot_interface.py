from abc import ABC, abstractmethod
from typing import List, Optional, Dict, Any
from dataclasses import dataclass
import numpy as np

@dataclass
class JointState:
    position: float
    velocity: float
    effort: float

@dataclass
class RobotState:
    joint_states: Dict[str, JointState]
    ee_pose: np.ndarray  # 4x4 homogeneous transformation matrix
    gripper_state: float

class RobotInterface(ABC):
    """Generic interface for any robotic arm."""
    
    @abstractmethod
    def initialize(self, config: Dict[str, Any]) -> bool:
        """Initialize the robot with given configuration."""
        pass
    
    @abstractmethod
    def get_joint_names(self) -> List[str]:
        """Get list of joint names."""
        pass
    
    @abstractmethod
    def get_robot_state(self) -> RobotState:
        """Get current robot state."""
        pass
    
    @abstractmethod
    def move_to_joint_positions(self, positions: Dict[str, float], speed: float = 1.0) -> bool:
        """Move to specified joint positions."""
        pass
    
    @abstractmethod
    def move_to_pose(self, target_pose: np.ndarray, speed: float = 1.0) -> bool:
        """Move end-effector to target pose."""
        pass
    
    @abstractmethod
    def set_gripper(self, state: float) -> bool:
        """Control gripper state (0=closed, 1=open)."""
        pass
    
    @abstractmethod
    def stop(self) -> bool:
        """Stop all robot motion immediately."""
        pass
    
    @abstractmethod
    def is_simulation(self) -> bool:
        """Check if this is a simulated robot."""
        pass
    
    @abstractmethod
    def get_robot_info(self) -> Dict[str, Any]:
        """Get robot specifications and capabilities."""
        pass

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