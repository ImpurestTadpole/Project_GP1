#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import argparse
import os
import yaml
import time
from datetime import datetime
from typing import Dict, List, Optional

# LeRobot imports
try:
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.common.policies.utils import get_support_policy_ids
    from lerobot.common.policies.policy import Policy
    from lerobot.common.envs.env import Env
    from lerobot.common.envs.utils import get_support_env_ids
    from lerobot.common.utils.utils import set_seed
    from lerobot.scripts.train import main as lerobot_train_main
    LEROBOT_AVAILABLE = True
except ImportError:
    LEROBOT_AVAILABLE = False
    print("Warning: LeRobot not available. Install with: pip install lerobot")

class LeRobotTrainer(Node):
    def __init__(self, config_path=None):
        super().__init__('lerobot_trainer')
        
        # Load training configuration
        self.config = self.load_config(config_path)
        
        # Initialize LeRobot components
        self.policy = None
        self.env = None
        self.dataset = None
        
        # Training state
        self.is_training = False
        self.current_episode = 0
        self.total_episodes = self.config.get('total_episodes', 1000)
        self.episode_length = self.config.get('episode_length', 300)
        
        # Publishers for training status
        self.status_pub = self.create_publisher(String, '/training/status', 10)
        self.metrics_pub = self.create_publisher(String, '/training/metrics', 10)
        
        # Subscribers for training control
        self.control_sub = self.create_subscription(
            String, '/training/control', self.control_callback, 10)
        
        # Timer for training loop
        self.training_timer = self.create_timer(1.0, self.training_loop)
        
        # Initialize training
        self.initialize_training()
        
        self.get_logger().info('LeRobot Trainer started')
        
    def load_config(self, config_path):
        """Load training configuration"""
        if config_path and os.path.exists(config_path):
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        else:
            return self.get_default_config()
            
    def get_default_config(self):
        """Get default training configuration"""
        return {
            'policy': {
                'type': 'ppo2',
                'policy_type': 'transformer',
                'learning_rate': 3e-4,
                'batch_size': 64,
                'epochs': 10,
                'gamma': 0.99,
                'gae_lambda': 0.95,
                'clip_ratio': 0.2,
                'value_loss_coef': 0.5,
                'entropy_coef': 0.01
            },
            'env': {
                'type': 'so101_dual_arm',
                'max_episode_length': 300,
                'action_space': 'continuous',
                'observation_space': 'multimodal'
            },
            'training': {
                'total_episodes': 1000,
                'episode_length': 300,
                'save_frequency': 100,
                'eval_frequency': 50,
                'log_frequency': 10
            },
            'data': {
                'dataset_path': '~/data/lerobot_episodes',
                'validation_split': 0.2,
                'augmentation': True
            },
            'output': {
                'model_dir': '~/models/lerobot',
                'log_dir': '~/logs/lerobot',
                'checkpoint_dir': '~/checkpoints/lerobot'
            }
        }
        
    def initialize_training(self):
        """Initialize training components"""
        if not LEROBOT_AVAILABLE:
            self.get_logger().error('LeRobot not available. Cannot initialize training.')
            return
            
        try:
            # Initialize policy
            policy_config = self.config['policy']
            self.policy = Policy(
                policy_id=policy_config['type'],
                policy_type=policy_config['policy_type'],
                **policy_config
            )
            
            # Initialize environment
            env_config = self.config['env']
            self.env = Env(
                env_id=env_config['type'],
                max_episode_length=env_config['max_episode_length'],
                action_space=env_config['action_space'],
                observation_space=env_config['observation_space']
            )
            
            # Load dataset
            dataset_path = os.path.expanduser(self.config['data']['dataset_path'])
            if os.path.exists(dataset_path):
                self.dataset = LeRobotDataset(dataset_path)
                self.get_logger().info(f'Loaded dataset from {dataset_path}')
            else:
                self.get_logger().warn(f'Dataset not found at {dataset_path}. Training without dataset.')
                
            # Create output directories
            for dir_key in ['model_dir', 'log_dir', 'checkpoint_dir']:
                dir_path = os.path.expanduser(self.config['output'][dir_key])
                os.makedirs(dir_path, exist_ok=True)
                
            self.get_logger().info('Training components initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize training: {e}')
            
    def control_callback(self, msg):
        """Handle training control commands"""
        command = msg.data.lower()
        
        if command == 'start':
            self.start_training()
        elif command == 'stop':
            self.stop_training()
        elif command == 'pause':
            self.pause_training()
        elif command == 'resume':
            self.resume_training()
        elif command == 'save':
            self.save_model()
        elif command == 'eval':
            self.evaluate_model()
        else:
            self.get_logger().warn(f'Unknown command: {command}')
            
    def start_training(self):
        """Start training"""
        if not self.policy or not self.env:
            self.get_logger().error('Training components not initialized')
            return
            
        self.is_training = True
        self.current_episode = 0
        self.get_logger().info('Training started')
        
    def stop_training(self):
        """Stop training"""
        self.is_training = False
        self.save_model()
        self.get_logger().info('Training stopped')
        
    def pause_training(self):
        """Pause training"""
        self.is_training = False
        self.get_logger().info('Training paused')
        
    def resume_training(self):
        """Resume training"""
        self.is_training = True
        self.get_logger().info('Training resumed')
        
    def training_loop(self):
        """Main training loop"""
        if not self.is_training:
            return
            
        if self.current_episode >= self.total_episodes:
            self.stop_training()
            return
            
        try:
            # Run one episode
            episode_reward = self.run_episode()
            
            # Update policy
            if self.dataset:
                self.update_policy()
                
            # Log metrics
            self.log_metrics(episode_reward)
            
            # Save checkpoint
            if self.current_episode % self.config['training']['save_frequency'] == 0:
                self.save_model()
                
            # Evaluate
            if self.current_episode % self.config['training']['eval_frequency'] == 0:
                self.evaluate_model()
                
            self.current_episode += 1
            
        except Exception as e:
            self.get_logger().error(f'Error in training loop: {e}')
            self.stop_training()
            
    def run_episode(self):
        """Run one training episode"""
        if not self.env:
            return 0.0
            
        # Reset environment
        observation = self.env.reset()
        episode_reward = 0.0
        
        for step in range(self.episode_length):
            # Get action from policy
            action = self.policy.predict(observation)
            
            # Execute action
            next_observation, reward, done, info = self.env.step(action)
            
            # Store experience (if using replay buffer)
            if hasattr(self.policy, 'store_experience'):
                self.policy.store_experience(observation, action, reward, next_observation, done)
                
            episode_reward += reward
            observation = next_observation
            
            if done:
                break
                
        return episode_reward
        
    def update_policy(self):
        """Update policy using collected data"""
        if not self.policy or not self.dataset:
            return
            
        try:
            # Get batch of experiences
            batch = self.dataset.sample_batch(self.config['policy']['batch_size'])
            
            # Update policy
            loss = self.policy.update(batch)
            
            # Log loss
            self.log_metrics({'policy_loss': loss})
            
        except Exception as e:
            self.get_logger().error(f'Error updating policy: {e}')
            
    def save_model(self):
        """Save trained model"""
        if not self.policy:
            return
            
        try:
            model_path = os.path.expanduser(self.config['output']['model_dir'])
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            model_file = os.path.join(model_path, f'model_episode_{self.current_episode}_{timestamp}.pth')
            
            self.policy.save(model_file)
            self.get_logger().info(f'Model saved to {model_file}')
            
        except Exception as e:
            self.get_logger().error(f'Error saving model: {e}')
            
    def evaluate_model(self):
        """Evaluate trained model"""
        if not self.policy or not self.env:
            return
            
        try:
            eval_episodes = 10
            eval_rewards = []
            
            for _ in range(eval_episodes):
                observation = self.env.reset()
                episode_reward = 0.0
                
                for step in range(self.episode_length):
                    action = self.policy.predict(observation)
                    observation, reward, done, _ = self.env.step(action)
                    episode_reward += reward
                    
                    if done:
                        break
                        
                eval_rewards.append(episode_reward)
                
            avg_reward = sum(eval_rewards) / len(eval_rewards)
            self.log_metrics({'eval_reward': avg_reward})
            self.get_logger().info(f'Evaluation average reward: {avg_reward:.2f}')
            
        except Exception as e:
            self.get_logger().error(f'Error evaluating model: {e}')
            
    def log_metrics(self, metrics):
        """Log training metrics"""
        if isinstance(metrics, dict):
            metrics_str = ', '.join([f'{k}: {v:.4f}' for k, v in metrics.items()])
        else:
            metrics_str = f'reward: {metrics:.4f}'
            
        # Publish metrics
        metrics_msg = String()
        metrics_msg.data = f'Episode {self.current_episode}: {metrics_str}'
        self.metrics_pub.publish(metrics_msg)
        
        # Log to file
        log_path = os.path.expanduser(self.config['output']['log_dir'])
        log_file = os.path.join(log_path, 'training.log')
        
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        with open(log_file, 'a') as f:
            f.write(f'{timestamp} - Episode {self.current_episode}: {metrics_str}\n')
            
    def publish_status(self):
        """Publish training status"""
        status_msg = String()
        
        if self.is_training:
            status_msg.data = f'TRAINING: Episode {self.current_episode}/{self.total_episodes}'
        else:
            status_msg.data = f'IDLE: Ready to train (Episode {self.current_episode})'
            
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    
    parser = argparse.ArgumentParser(description='Train SO-101 robot with LeRobot')
    parser.add_argument('--config', help='Path to training configuration file')
    
    args, unknown = parser.parse_known_args()
    
    node = LeRobotTrainer(config_path=args.config)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.is_training:
            node.stop_training()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 