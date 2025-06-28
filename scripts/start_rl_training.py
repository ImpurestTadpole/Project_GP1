#!/usr/bin/env python3
"""
HIL-SERL Training Launcher
Starts the Human-in-the-Loop Sample-Efficient Reinforcement Learning training process
"""

import subprocess
import sys
import os
import signal
import time
import argparse
import threading
from pathlib import Path
import yaml


def signal_handler(sig, frame):
    """Handle Ctrl+C to gracefully shutdown"""
    print("\nShutting down HIL-SERL training...")
    sys.exit(0)


def check_dependencies():
    """Check if required dependencies are available"""
    print("Checking HIL-SERL dependencies...")
    
    checks = [
        ("ROS 2", "ros2 --version"),
        ("PyTorch", "python3 -c 'import torch; print(f\"PyTorch {torch.__version__}\")'"),
        ("Gymnasium", "python3 -c 'import gymnasium; print(\"Gymnasium available\")'"),
        ("Stable Baselines3", "python3 -c 'import stable_baselines3; print(\"SB3 available\")'"),
        ("Workspace", "ls ~/ros_ws/src/so101_lerobot")
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


def check_demonstration_data():
    """Check if demonstration data is available"""
    print("Checking demonstration data...")
    
    data_paths = [
        "~/data/sim_episodes",
        "~/data/real_episodes", 
        "~/data/lerobot_episodes"
    ]
    
    total_episodes = 0
    for path in data_paths:
        expanded_path = os.path.expanduser(path)
        if os.path.exists(expanded_path):
            episodes = len([f for f in os.listdir(expanded_path) if f.startswith("episode_")])
            if episodes > 0:
                print(f"  ✓ {path}: {episodes} episodes")
                total_episodes += episodes
    
    if total_episodes > 0:
        print(f"Total demonstration episodes: {total_episodes}")
        return True
    else:
        print("No demonstration data found")
        print("Please record some episodes first using VR teleoperation")
        return False


def launch_rl_components(config_path, mode="sim"):
    """Launch all RL training components"""
    print("Launching HIL-SERL components...")
    
    workspace_path = os.path.expanduser("~/ros_ws")
    processes = {}
    
    try:
        # 1. Launch HIL-SERL trainer
        print("  Starting HIL-SERL trainer...")
        trainer_cmd = f"""
        cd {workspace_path} && \
        source install/setup.bash && \
        python3 src/so101_lerobot/lerobot/hil_serl_trainer.py --config {config_path}
        """
        
        processes['trainer'] = subprocess.Popen(
            trainer_cmd, shell=True, executable="/bin/bash"
        )
        time.sleep(3)
        
        # 2. Launch intervention handler
        print("  Starting intervention handler...")
        intervention_cmd = f"""
        cd {workspace_path} && \
        source install/setup.bash && \
        python3 src/so101_lerobot/lerobot/intervention_handler.py
        """
        
        processes['intervention'] = subprocess.Popen(
            intervention_cmd, shell=True, executable="/bin/bash"
        )
        time.sleep(2)
        
        # 3. Launch environment (sim or real)
        if mode == "sim":
            print("  Starting simulation environment...")
            sim_cmd = f"""
            cd {workspace_path} && \
            source install/setup.bash && \
            ros2 launch so101_sim sim_launch.py use_sim_time:=true gui:=true rviz:=false
            """
            
            processes['environment'] = subprocess.Popen(
                sim_cmd, shell=True, executable="/bin/bash"
            )
        else:
            print("  Starting real robot environment...")
            real_cmd = f"""
            cd {workspace_path} && \
            source install/setup.bash && \
            ros2 launch so101_bringup real.launch.py
            """
            
            processes['environment'] = subprocess.Popen(
                real_cmd, shell=True, executable="/bin/bash"
            )
        
        time.sleep(5)
        
        # 4. Launch camera (for real mode)
        if mode == "real":
            print("  Starting camera...")
            camera_cmd = f"""
            cd {workspace_path} && \
            source install/setup.bash && \
            ros2 launch so101_camera camera.launch.py camera_type:=usb
            """
            
            processes['camera'] = subprocess.Popen(
                camera_cmd, shell=True, executable="/bin/bash"
            )
            time.sleep(2)
        
        print("All components launched successfully!")
        print("\nHIL-SERL Training Status:")
        print("========================")
        print("- HIL-SERL Trainer: Running")
        print("- Intervention Handler: Running") 
        print(f"- Environment: {mode.capitalize()}")
        print("\nVR Controller Controls:")
        print("- A Button: Start/Stop Intervention")
        print("- B Button: Positive Feedback")
        print("- X Button: Negative Feedback")
        print("\nPress Ctrl+C to stop training")
        
        # Monitor processes
        while True:
            # Check if any process died
            for name, process in processes.items():
                if process.poll() is not None:
                    print(f"\n{name} process terminated unexpectedly")
                    return False
            
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nShutting down HIL-SERL training...")
        
        # Terminate all processes
        for name, process in processes.items():
            print(f"Stopping {name}...")
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
        
        return True
    
    except Exception as e:
        print(f"Error launching components: {e}")
        return False


def create_default_config():
    """Create default RL config if it doesn't exist"""
    config_path = os.path.expanduser("~/ros_ws/config/rl_config.yaml")
    
    if not os.path.exists(config_path):
        print(f"Creating default config at {config_path}")
        
        # Create config directory if it doesn't exist
        os.makedirs(os.path.dirname(config_path), exist_ok=True)
        
        # This would copy from the repository's config
        # For now, just notify the user
        print("Please ensure rl_config.yaml exists in ~/ros_ws/config/")
        return False
    
    return True


def main():
    """Main function"""
    signal.signal(signal.SIGINT, signal_handler)
    
    parser = argparse.ArgumentParser(description="Launch HIL-SERL training")
    parser.add_argument(
        "--config", 
        type=str, 
        default="~/ros_ws/config/rl_config.yaml",
        help="Path to RL configuration file"
    )
    parser.add_argument(
        "--mode",
        type=str,
        choices=["sim", "real"],
        default="sim", 
        help="Training mode: simulation or real robot"
    )
    parser.add_argument(
        "--skip-checks",
        action="store_true",
        help="Skip dependency and data checks"
    )
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("SO-101 HIL-SERL Training Launcher")
    print("=" * 60)
    
    # Expand config path
    config_path = os.path.expanduser(args.config)
    
    if not args.skip_checks:
        # Check dependencies
        if not check_dependencies():
            print("Dependency check failed. Please install missing packages.")
            return 1
        
        # Check demonstration data
        if not check_demonstration_data():
            print("No demonstration data found. Please collect demonstrations first.")
            return 1
        
        # Check config file
        if not create_default_config():
            print("Configuration setup failed.")
            return 1
    
    # Launch training
    success = launch_rl_components(config_path, args.mode)
    
    if success:
        print("HIL-SERL training completed successfully!")
        return 0
    else:
        print("HIL-SERL training failed.")
        return 1


if __name__ == "__main__":
    sys.exit(main()) 