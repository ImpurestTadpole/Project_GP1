#!/usr/bin/env python3
"""
Simple RL Trainer for SO-101 System
Uses stable-baselines3 for reinforcement learning without complex dependencies
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float32, Bool
import numpy as np
import torch
import gymnasium as gym
from stable_baselines3 import SAC, PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import BaseCallback
import os
import yaml
import time
from collections import deque
import threading


class ROS2GymWrapper(gym.Env):
    """Gymnasium wrapper for ROS 2 environment"""
    
    def __init__(self, node):
        super().__init__()
        self.node = node
        
        # Define observation and action spaces
        # Observation: image features (64) + joint states (14)
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(78,), dtype=np.float32
        )
        
        # Action: dual arm poses (14 DOF)
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(14,), dtype=np.float32
        )
        
        self.reset()
    
    def reset(self, seed=None, options=None):
        """Reset the environment"""
        super().reset(seed=seed)
        
        # Wait for initial observations
        while self.node.latest_observation is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        observation = self.node.latest_observation
        info = {}
        
        return observation, info
    
    def step(self, action):
        """Execute action and return next observation, reward, done, info"""
        # Send action to robot
        self.node.publish_action(action)
        
        # Wait for next observation
        start_time = time.time()
        while time.time() - start_time < 0.1:  # 10 Hz
            rclpy.spin_once(self.node, timeout_sec=0.01)
        
        # Get observation
        observation = self.node.latest_observation
        
        # Get reward (simple distance-based reward for now)
        reward = self.node.compute_reward()
        
        # Check if episode is done
        done = self.node.is_episode_done()
        
        info = {}
        
        return observation, reward, done, False, info


class SimpleRLTrainer(Node):
    """Simple RL trainer using stable-baselines3"""
    
    def __init__(self, config_path="config/rl_config.yaml"):
        super().__init__('simple_rl_trainer')
        
        # Load configuration
        self.config = self.load_config(config_path)
        
        # State variables
        self.latest_image = None
        self.latest_joints = None
        self.latest_observation = None
        self.episode_start_time = time.time()
        self.step_count = 0
        
        # Target pose for simple reward computation
        self.target_poses = None
        
        # Setup ROS interface
        self.setup_ros_interface()
        
        # Create gym environment
        self.env = ROS2GymWrapper(self)
        
        # Initialize RL algorithm
        self.setup_rl_algorithm()
        
        self.get_logger().info("Simple RL Trainer initialized")
    
    def load_config(self, config_path):
        """Load configuration file"""
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            return config
        except FileNotFoundError:
            self.get_logger().warn(f"Config file {config_path} not found, using defaults")
            return self.get_default_config()
    
    def get_default_config(self):
        """Default configuration"""
        return {
            'algorithm': 'SAC',
            'total_timesteps': 100000,
            'learning_rate': 3e-4,
            'batch_size': 256,
            'buffer_size': 100000,
            'learning_starts': 1000,
            'save_freq': 10000,
            'log_interval': 10
        }
    
    def setup_ros_interface(self):
        """Setup ROS publishers and subscribers"""
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw',
            self.image_callback, 10
        )
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states',
            self.joint_callback, 10
        )
        
        # Publishers
        self.action_pub = self.create_publisher(
            PoseArray, '/so101_teleop/cmd_pose', 10
        )
        self.reward_pub = self.create_publisher(
            Float32, '/rl/reward', 10
        )
    
    def setup_rl_algorithm(self):
        """Initialize the RL algorithm"""
        algorithm = self.config.get('algorithm', 'SAC')
        
        if algorithm == 'SAC':
            self.model = SAC(
                'MlpPolicy',
                self.env,
                learning_rate=self.config.get('learning_rate', 3e-4),
                buffer_size=self.config.get('buffer_size', 100000),
                batch_size=self.config.get('batch_size', 256),
                learning_starts=self.config.get('learning_starts', 1000),
                verbose=1,
                device='cuda' if torch.cuda.is_available() else 'cpu'
            )
        elif algorithm == 'PPO':
            self.model = PPO(
                'MlpPolicy',
                self.env,
                learning_rate=self.config.get('learning_rate', 3e-4),
                n_steps=2048,
                batch_size=self.config.get('batch_size', 64),
                verbose=1,
                device='cuda' if torch.cuda.is_available() else 'cpu'
            )
        else:
            raise ValueError(f"Unsupported algorithm: {algorithm}")
    
    def image_callback(self, msg):
        """Process camera images"""
        # For now, just extract simple features
        # In practice, you'd use a CNN encoder
        self.latest_image = np.random.randn(64)  # Placeholder
        self.update_observation()
    
    def joint_callback(self, msg):
        """Process joint states"""
        if len(msg.position) >= 14:
            self.latest_joints = np.array(msg.position[:14], dtype=np.float32)
            self.update_observation()
    
    def update_observation(self):
        """Combine image and joint data into observation"""
        if self.latest_image is not None and self.latest_joints is not None:
            self.latest_observation = np.concatenate([
                self.latest_image,  # 64 features
                self.latest_joints  # 14 joint positions
            ])
    
    def publish_action(self, action):
        """Convert RL action to ROS message and publish"""
        # Convert normalized action [-1, 1] to pose array
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "base_link"
        
        # Simple conversion - in practice you'd have proper pose conversion
        # This is a placeholder implementation
        from geometry_msgs.msg import Pose
        
        left_pose = Pose()
        left_pose.position.x = float(action[0] * 0.5)
        left_pose.position.y = float(action[1] * 0.5)
        left_pose.position.z = float(action[2] * 0.5 + 0.3)
        left_pose.orientation.w = 1.0
        
        right_pose = Pose()
        right_pose.position.x = float(action[7] * 0.5)
        right_pose.position.y = float(action[8] * 0.5)
        right_pose.position.z = float(action[9] * 0.5 + 0.3)
        right_pose.orientation.w = 1.0
        
        pose_array.poses = [left_pose, right_pose]
        self.action_pub.publish(pose_array)
        
        self.step_count += 1
    
    def compute_reward(self):
        """Compute reward for current state"""
        # Simple reward based on time (encourages longer episodes)
        # In practice, you'd compute task-specific rewards
        base_reward = 0.1
        
        # Penalty for very long episodes
        episode_length = time.time() - self.episode_start_time
        if episode_length > 30.0:  # 30 seconds max
            base_reward -= 1.0
        
        # Publish reward for monitoring
        reward_msg = Float32()
        reward_msg.data = base_reward
        self.reward_pub.publish(reward_msg)
        
        return base_reward
    
    def is_episode_done(self):
        """Check if episode should end"""
        episode_length = time.time() - self.episode_start_time
        return episode_length > 30.0 or self.step_count > 1000
    
    def train(self):
        """Start training the RL policy"""
        self.get_logger().info("Starting RL training...")
        
        try:
            # Train the model
            self.model.learn(
                total_timesteps=self.config.get('total_timesteps', 100000),
                log_interval=self.config.get('log_interval', 10),
                callback=self.create_callback()
            )
            
            # Save the trained model
            model_path = os.path.expanduser("~/models/simple_rl_model")
            os.makedirs(os.path.dirname(model_path), exist_ok=True)
            self.model.save(model_path)
            
            self.get_logger().info(f"Training completed! Model saved to {model_path}")
            
        except KeyboardInterrupt:
            self.get_logger().info("Training interrupted by user")
        except Exception as e:
            self.get_logger().error(f"Training failed: {e}")
    
    def create_callback(self):
        """Create training callback for monitoring"""
        class TrainingCallback(BaseCallback):
            def __init__(self, node):
                super().__init__()
                self.node = node
            
            def _on_step(self) -> bool:
                if self.n_calls % 1000 == 0:
                    self.node.get_logger().info(f"Training step: {self.n_calls}")
                return True
        
        return TrainingCallback(self)


def main(args=None):
    rclpy.init(args=args)
    
    trainer = SimpleRLTrainer()
    
    # Run training in a separate thread
    training_thread = threading.Thread(target=trainer.train)
    training_thread.daemon = True
    training_thread.start()
    
    try:
        rclpy.spin(trainer)
    except KeyboardInterrupt:
        trainer.get_logger().info("Shutting down...")
    finally:
        trainer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 