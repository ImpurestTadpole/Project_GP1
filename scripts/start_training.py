#!/usr/bin/env python3

import subprocess
import sys
import os
import signal
import time
import argparse
from pathlib import Path

def signal_handler(sig, frame):
    """Handle Ctrl+C to gracefully shutdown"""
    print("\nShutting down training...")
    sys.exit(0)

def check_dependencies():
    """Check if required dependencies are available"""
    print("Checking dependencies...")
    
    checks = [
        ("ROS 2", "ros2 --version"),
        ("LeRobot", "python3 -c 'import lerobot; print(\"LeRobot available\")'"),
        ("PyTorch", "python3 -c 'import torch; print(f\"PyTorch {torch.__version__}\")'"),
        ("Workspace", "ls ~/ros_ws/src/lerobot")
    ]
    
    for name, command in checks:
        try:
            result = subprocess.run(command, shell=True, capture_output=True, text=True)
            if result.returncode == 0:
                print(f"✓ {name}: OK")
            else:
                print(f"✗ {name}: Failed")
                return False
        except Exception as e:
            print(f"✗ {name}: Error - {e}")
            return False
    
    return True

def check_data_availability():
    """Check if training data is available"""
    print("Checking training data...")
    
    data_paths = [
        "~/data/sim_episodes",
        "~/data/real_episodes",
        "~/data/lerobot_episodes"
    ]
    
    available_data = []
    for path in data_paths:
        expanded_path = os.path.expanduser(path)
        if os.path.exists(expanded_path):
            # Count episodes
            episodes = len([f for f in os.listdir(expanded_path) if f.startswith("episode_")])
            if episodes > 0:
                available_data.append(f"{path}: {episodes} episodes")
    
    if available_data:
        print("Training data found:")
        for data in available_data:
            print(f"  ✓ {data}")
        return True
    else:
        print("No training data found")
        print("Please record some episodes first using the data collection script")
        return False

def launch_training(config_path=None):
    """Launch training"""
    print("Launching LeRobot training...")
    
    workspace_path = os.path.expanduser("~/ros_ws")
    
    # Build command
    if config_path:
        command = f"""
        cd {workspace_path} && \
        source install/setup.bash && \
        python3 src/lerobot/lerobot/train.py --config {config_path}
        """
    else:
        command = f"""
        cd {workspace_path} && \
        source install/setup.bash && \
        python3 src/lerobot/lerobot/train.py
        """
    
    try:
        subprocess.run(command, shell=True, executable="/bin/bash")
    except KeyboardInterrupt:
        print("\nTraining interrupted by user")
    except Exception as e:
        print(f"Error launching training: {e}")
        return False
    
    return True

def launch_data_collection():
    """Launch data collection for training"""
    print("Launching data collection...")
    
    workspace_path = os.path.expanduser("~/ros_ws")
    
    command = f"""
    cd {workspace_path} && \
    source install/setup.bash && \
    python3 src/lerobot/lerobot/control_robot.py --mode record
    """
    
    try:
        subprocess.run(command, shell=True, executable="/bin/bash")
    except KeyboardInterrupt:
        print("\nData collection interrupted by user")
    except Exception as e:
        print(f"Error launching data collection: {e}")
        return False
    
    return True

def launch_evaluation(model_path):
    """Launch model evaluation"""
    print("Launching model evaluation...")
    
    workspace_path = os.path.expanduser("~/ros_ws")
    
    command = f"""
    cd {workspace_path} && \
    source install/setup.bash && \
    python3 src/lerobot/lerobot/control_robot.py --mode eval --policy-id {model_path}
    """
    
    try:
        subprocess.run(command, shell=True, executable="/bin/bash")
    except KeyboardInterrupt:
        print("\nEvaluation interrupted by user")
    except Exception as e:
        print(f"Error launching evaluation: {e}")
        return False
    
    return True

def main():
    parser = argparse.ArgumentParser(description="Launcher for the LeRobot training script.")
    parser.add_argument(
        "--config",
        type=str,
        default="config/lerobot_config.yaml",
        help="Path to the training configuration file for LeRobot.",
    )
    parser.add_argument(
        '--opts',
        nargs='*',
        default=[],
        help="Additional options to pass to the LeRobot train script (e.g., 'train.batch_size=32')."
    )
    args = parser.parse_args()

    # Get the absolute path to the LeRobot training script
    lerobot_train_script = os.path.expanduser("~/lerobot/lerobot/scripts/train.py")

    if not os.path.exists(lerobot_train_script):
        print(f"Error: LeRobot training script not found at {lerobot_train_script}")
        print("Please ensure LeRobot is cloned into your home directory.")
        return

    if not os.path.exists(args.config):
        print(f"Error: Training config not found at {args.config}")
        print("Please ensure the configuration file exists.")
        return

    # Construct the command to run the training script
    command = [
        "python",
        lerobot_train_script,
        f"--config-path={os.path.dirname(args.config)}",
        f"--config-name={os.path.basename(args.config)}",
    ]
    
    # Add any extra options
    if args.opts:
        command.extend(args.opts)

    print("Launching LeRobot training with command:")
    print(" ".join(command))
    
    try:
        # Execute the command
        subprocess.run(command, check=True)
    except subprocess.CalledProcessError as e:
        print(f"LeRobot training script failed with exit code {e.returncode}")
    except FileNotFoundError:
        print("Error: 'python' command not found. Is Python installed and in your PATH?")

if __name__ == "__main__":
    main() 