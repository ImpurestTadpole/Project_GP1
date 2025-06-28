from typing import Dict, Any, List
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction
from ..robot_interface import RobotInterface, RobotState, JointState

class IsaacSimInterface(RobotInterface):
    """Interface for controlling robots in NVIDIA Isaac Sim."""
    
    def __init__(self):
        self.world = None
        self.robot = None
        self.robot_path = None
        self.joint_names = []
        
    def initialize(self, config: Dict[str, Any]) -> bool:
        """Initialize Isaac Sim and load robot."""
        try:
            # Initialize Isaac Sim world
            self.world = World(physics_dt=1/60.0)
            
            # Load robot URDF/USD
            self.robot_path = config["robot_path"]
            self.robot = self.world.scene.add(
                Robot(
                    prim_path=self.robot_path,
                    name=config["robot_name"],
                    position=config.get("position", [0, 0, 0])
                )
            )
            
            # Get joint configuration
            self.joint_names = self.robot.dof_names
            
            # Initialize physics
            self.world.reset()
            return True
            
        except Exception as e:
            print(f"Failed to initialize Isaac Sim: {e}")
            return False
    
    def get_joint_names(self) -> List[str]:
        """Get list of joint names."""
        return self.joint_names
    
    def get_robot_state(self) -> RobotState:
        """Get current robot state."""
        # Get joint states
        positions = self.robot.get_joint_positions()
        velocities = self.robot.get_joint_velocities()
        efforts = self.robot.get_applied_joint_efforts()
        
        joint_states = {}
        for i, name in enumerate(self.joint_names):
            joint_states[name] = JointState(
                position=positions[i],
                velocity=velocities[i],
                effort=efforts[i]
            )
            
        # Get end-effector pose
        ee_pose = self.robot.end_effector.get_world_pose()
        
        return RobotState(
            joint_states=joint_states,
            ee_pose=ee_pose,
            gripper_state=self.robot.gripper.get_state() if hasattr(self.robot, "gripper") else 0.0
        )
    
    def move_to_joint_positions(self, positions: Dict[str, float], speed: float = 1.0) -> bool:
        """Move to specified joint positions."""
        try:
            # Convert dict to array matching joint order
            pos_array = [positions[name] for name in self.joint_names]
            
            # Create articulation action
            action = ArticulationAction(
                joint_positions=pos_array,
                joint_velocities=None,
                joint_efforts=None
            )
            
            # Apply action
            self.robot.apply_action(action)
            return True
            
        except Exception as e:
            print(f"Failed to move to joint positions: {e}")
            return False
    
    def move_to_pose(self, target_pose: np.ndarray, speed: float = 1.0) -> bool:
        """Move end-effector to target pose using inverse kinematics."""
        try:
            # Use Isaac Sim's IK solver
            joint_positions = self.robot.compute_inverse_kinematics(target_pose)
            return self.move_to_joint_positions(
                dict(zip(self.joint_names, joint_positions)),
                speed
            )
        except Exception as e:
            print(f"Failed to move to pose: {e}")
            return False
    
    def set_gripper(self, state: float) -> bool:
        """Control gripper state."""
        if hasattr(self.robot, "gripper"):
            try:
                self.robot.gripper.set_state(state)
                return True
            except Exception as e:
                print(f"Failed to set gripper state: {e}")
        return False
    
    def stop(self) -> bool:
        """Stop all robot motion."""
        try:
            zero_action = ArticulationAction(
                joint_positions=self.robot.get_joint_positions(),
                joint_velocities=[0.0] * len(self.joint_names),
                joint_efforts=[0.0] * len(self.joint_names)
            )
            self.robot.apply_action(zero_action)
            return True
        except Exception as e:
            print(f"Failed to stop robot: {e}")
            return False
    
    def is_simulation(self) -> bool:
        """This is always a simulation."""
        return True
    
    def get_robot_info(self) -> Dict[str, Any]:
        """Get robot specifications."""
        return {
            "name": self.robot.name if self.robot else None,
            "num_joints": len(self.joint_names),
            "joint_names": self.joint_names,
            "has_gripper": hasattr(self.robot, "gripper"),
            "simulation_backend": "isaac_sim"
        } 