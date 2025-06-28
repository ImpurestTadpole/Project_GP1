#!/usr/bin/env python3
"""
HIL-SERL (Human-in-the-Loop Sample-Efficient Reinforcement Learning) Trainer
Integrates human demonstrations from VR teleoperation with RL policy training
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float32, Bool
import numpy as np
import torch
import torch.nn as nn
import gymnasium as gym
from collections import deque
import threading
import time
import yaml
import os
from pathlib import Path

try:
    import agentlace
except ImportError:
    print("Warning: agentlace not installed. Using fallback implementation.")
    print("For full HIL-SERL functionality, install with:")
    print("pip install git+https://github.com/youliangtan/agentlace.git")
    agentlace = None

try:
    import wandb
except ImportError:
    print("Warning: wandb not installed. Install with: pip install wandb")
    wandb = None


class RewardClassifier(nn.Module):
    """Neural network to classify human preferences into reward signals"""
    
    def __init__(self, obs_dim, action_dim, hidden_dim=256):
        super().__init__()
        self.network = nn.Sequential(
            nn.Linear(obs_dim + action_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1),
            nn.Sigmoid()
        )
    
    def forward(self, obs, action):
        x = torch.cat([obs, action], dim=-1)
        return self.network(x)


class HILSERLTrainer(Node):
    """Main HIL-SERL training node"""
    
    def __init__(self, config_path="config/rl_config.yaml"):
        super().__init__('hil_serl_trainer')
        
        # Load configuration
        self.config = self.load_config(config_path)
        
        # Initialize components
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.setup_logging()
        self.setup_environment()
        self.setup_reward_classifier()
        self.setup_rl_algorithm()
        
        # ROS subscribers and publishers
        self.setup_ros_interface()
        
        # Data buffers
        self.demo_buffer = deque(maxlen=self.config['demo_buffer_size'])
        self.rl_buffer = deque(maxlen=self.config['rl_buffer_size'])
        self.preference_buffer = deque(maxlen=self.config['preference_buffer_size'])
        
        # Training state
        self.training_active = False
        self.intervention_mode = False
        self.current_episode = 0
        
        # Start training loop in separate thread
        self.training_thread = threading.Thread(target=self.training_loop)
        self.training_thread.daemon = True
        self.training_thread.start()
        
        self.get_logger().info("HIL-SERL Trainer initialized")

    def load_config(self, config_path):
        """Load training configuration"""
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            return config
        except FileNotFoundError:
            self.get_logger().warn(f"Config file {config_path} not found, using defaults")
            return self.get_default_config()

    def get_default_config(self):
        """Default configuration for HIL-SERL training"""
        return {
            'demo_buffer_size': 10000,
            'rl_buffer_size': 100000,
            'preference_buffer_size': 5000,
            'reward_classifier_lr': 3e-4,
            'rl_learning_rate': 3e-4,
            'batch_size': 256,
            'reward_update_freq': 100,
            'policy_update_freq': 1,
            'intervention_threshold': 0.5,
            'max_episodes': 1000,
            'episode_length': 1000,
            'reward_classifier_epochs': 10,
            'use_wandb': True,
            'project_name': 'so101_hil_serl'
        }

    def setup_logging(self):
        """Initialize experiment logging"""
        if wandb and self.config.get('use_wandb', False):
            wandb.init(
                project=self.config['project_name'],
                config=self.config,
                name=f"hil_serl_{int(time.time())}"
            )

    def setup_environment(self):
        """Set up the RL environment interface"""
        # This would typically wrap your ROS environment in a Gymnasium interface
        # For now, we'll create a placeholder
        self.obs_dim = 64  # Image features + joint states
        self.action_dim = 14  # 7 DOF per arm
        
    def setup_reward_classifier(self):
        """Initialize the reward classifier network"""
        self.reward_classifier = RewardClassifier(
            self.obs_dim, 
            self.action_dim
        ).to(self.device)
        
        self.reward_optimizer = torch.optim.Adam(
            self.reward_classifier.parameters(),
            lr=self.config['reward_classifier_lr']
        )
        
    def setup_rl_algorithm(self):
        """Initialize the RL algorithm (SAC, TD3, etc.)"""
        # Placeholder for RL algorithm initialization
        # You would typically use stable-baselines3 or similar here
        self.policy = None  # Initialize your RL policy here
        
    def setup_ros_interface(self):
        """Set up ROS publishers and subscribers"""
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', 
            self.image_callback, 10
        )
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states',
            self.joint_callback, 10
        )
        self.intervention_sub = self.create_subscription(
            Bool, '/hil_serl/intervention',
            self.intervention_callback, 10
        )
        self.preference_sub = self.create_subscription(
            Float32, '/hil_serl/preference',
            self.preference_callback, 10
        )
        
        # Publishers
        self.action_pub = self.create_publisher(
            PoseArray, '/so101_teleop/cmd_pose', 10
        )
        self.reward_pub = self.create_publisher(
            Float32, '/hil_serl/reward', 10
        )
        self.training_status_pub = self.create_publisher(
            Bool, '/hil_serl/training_active', 10
        )

    def image_callback(self, msg):
        """Process incoming camera images"""
        # Convert ROS image to numpy array and extract features
        pass

    def joint_callback(self, msg):
        """Process joint state information"""
        # Store current joint positions
        pass

    def intervention_callback(self, msg):
        """Handle human intervention signals"""
        self.intervention_mode = msg.data
        if self.intervention_mode:
            self.get_logger().info("Human intervention activated")

    def preference_callback(self, msg):
        """Handle human preference feedback"""
        preference_score = msg.data
        # Store preference data for reward classifier training
        self.preference_buffer.append({
            'score': preference_score,
            'timestamp': time.time()
        })

    def training_loop(self):
        """Main training loop running in separate thread"""
        while rclpy.ok():
            if self.training_active and len(self.demo_buffer) > 0:
                # Train reward classifier
                if len(self.preference_buffer) > self.config['batch_size']:
                    self.train_reward_classifier()
                
                # Train RL policy
                if len(self.rl_buffer) > self.config['batch_size']:
                    self.train_rl_policy()
                
                # Publish training status
                status_msg = Bool()
                status_msg.data = True
                self.training_status_pub.publish(status_msg)
                
            time.sleep(0.1)

    def train_reward_classifier(self):
        """Train the reward classifier on human preference data"""
        if len(self.preference_buffer) < self.config['batch_size']:
            return
            
        # Sample batch from preference buffer
        batch_indices = np.random.choice(
            len(self.preference_buffer), 
            self.config['batch_size'], 
            replace=False
        )
        
        # Training logic for reward classifier
        for epoch in range(self.config['reward_classifier_epochs']):
            # Implement reward classifier training
            pass
        
        self.get_logger().info("Reward classifier updated")

    def train_rl_policy(self):
        """Train the RL policy using generated rewards"""
        if self.policy is None:
            return
            
        # Implement RL policy training
        # This would typically use your chosen RL algorithm
        pass

    def start_training(self):
        """Start the HIL-SERL training process"""
        self.training_active = True
        self.get_logger().info("HIL-SERL training started")

    def stop_training(self):
        """Stop the HIL-SERL training process"""
        self.training_active = False
        self.get_logger().info("HIL-SERL training stopped")

    def save_models(self, checkpoint_dir):
        """Save trained models"""
        os.makedirs(checkpoint_dir, exist_ok=True)
        
        # Save reward classifier
        torch.save(
            self.reward_classifier.state_dict(),
            os.path.join(checkpoint_dir, 'reward_classifier.pth')
        )
        
        # Save RL policy
        if self.policy is not None:
            # Save policy checkpoint
            pass
            
        self.get_logger().info(f"Models saved to {checkpoint_dir}")

    def load_models(self, checkpoint_dir):
        """Load pre-trained models"""
        reward_path = os.path.join(checkpoint_dir, 'reward_classifier.pth')
        if os.path.exists(reward_path):
            self.reward_classifier.load_state_dict(torch.load(reward_path))
            self.get_logger().info("Reward classifier loaded")


def main(args=None):
    rclpy.init(args=args)
    
    trainer = HILSERLTrainer()
    
    try:
        rclpy.spin(trainer)
    except KeyboardInterrupt:
        trainer.get_logger().info("Training interrupted by user")
    finally:
        trainer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 