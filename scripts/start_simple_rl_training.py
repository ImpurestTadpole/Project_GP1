#!/usr/bin/env python3
"""
Simple RL Training Launcher
Uses stable-baselines3 for basic reinforcement learning without complex dependencies
"""

import subprocess
import sys
import os
import signal
import time
import argparse
from pathlib import Path


def signal_handler(sig, frame):
    """Handle Ctrl+C to gracefully shutdown"""
    print("\nShutting down Simple RL training...")
    sys.exit(0)


def check_dependencies():
    """Check if required dependencies are available"""
    print("Checking Simple RL dependencies...")
    
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


def launch_simple_rl_training(mode="sim"):
    """Launch simple RL training components"""
    print("Launching Simple RL training components...")
    
    workspace_path = os.path.expanduser("~/ros_ws")
    processes = {}
    
    try:
        # 1. Launch environment (sim or real)
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
        
        # 2. Launch camera (for real mode)
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
        
        # 3. Launch Simple RL trainer
        print("  Starting Simple RL trainer...")
        trainer_cmd = f"""
        cd {workspace_path} && \
        source install/setup.bash && \
        python3 src/so101_lerobot/lerobot/simple_rl_trainer.py
        """
        
        processes['trainer'] = subprocess.Popen(
            trainer_cmd, shell=True, executable="/bin/bash"
        )
        time.sleep(3)
        
        print("All components launched successfully!")
        print("\nSimple RL Training Status:")
        print("=========================")
        print("- Simple RL Trainer: Running")
        print(f"- Environment: {mode.capitalize()}")
        print("- Algorithm: SAC (Soft Actor-Critic)")
        print("\nTraining will run automatically.")
        print("Monitor progress in the trainer terminal output.")
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
        print("\nShutting down Simple RL training...")
        
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


def main():
    """Main function"""
    signal.signal(signal.SIGINT, signal_handler)
    
    parser = argparse.ArgumentParser(description="Launch Simple RL training")
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
        help="Skip dependency checks"
    )
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("SO-101 Simple RL Training Launcher")
    print("Uses stable-baselines3 (no agentlace required)")
    print("=" * 60)
    
    if not args.skip_checks:
        # Check dependencies
        if not check_dependencies():
            print("Dependency check failed. Please install missing packages.")
            print("\nTo install missing packages:")
            print("conda activate lerobot  # if using conda environment")
            print("pip install stable-baselines3 gymnasium")
            return 1
    
    # Launch training
    success = launch_simple_rl_training(args.mode)
    
    if success:
        print("Simple RL training completed successfully!")
        return 0
    else:
        print("Simple RL training failed.")
        return 1


if __name__ == "__main__":
    sys.exit(main()) 