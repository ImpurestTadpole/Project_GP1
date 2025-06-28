#!/usr/bin/env python3

import subprocess
import sys
import os
import signal
import time
from pathlib import Path

def signal_handler(sig, frame):
    """Handle Ctrl+C to gracefully shutdown"""
    print("\nShutting down simulation...")
    sys.exit(0)

def check_dependencies():
    """Check if required dependencies are available"""
    print("Checking dependencies...")
    
    checks = [
        ("ROS 2", "ros2 --version"),
        ("Gazebo", "gazebo --version"),
        ("Workspace", "ls ~/ros_ws/src/so101_sim")
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

def source_environment():
    """Source ROS and workspace environment"""
    workspace_path = os.path.expanduser("~/ros_ws")
    
    # Source ROS 2
    os.environ["ROS_DISTRO"] = "humble"
    os.environ["ROS_VERSION"] = "2"
    
    # Source workspace
    setup_script = os.path.join(workspace_path, "install", "setup.bash")
    if os.path.exists(setup_script):
        subprocess.run(f"source {setup_script}", shell=True, executable="/bin/bash")

def launch_simulation():
    """Launch the simulation"""
    print("Launching SO-101 simulation...")
    
    workspace_path = os.path.expanduser("~/ros_ws")
    
    # Launch simulation
    command = f"""
    cd {workspace_path} && \
    source install/setup.bash && \
    ros2 launch so101_sim sim_launch.py \
        use_sim_time:=true \
        gui:=true \
        rviz:=true \
        use_planning:=true
    """
    
    try:
        subprocess.run(command, shell=True, executable="/bin/bash")
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
    except Exception as e:
        print(f"Error launching simulation: {e}")
        return False
    
    return True

def main():
    """Main function"""
    print("SO-101 Simulation Launcher")
    print("=" * 30)
    
    # Set up signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Check dependencies
    if not check_dependencies():
        print("Dependency check failed. Please install missing dependencies.")
        sys.exit(1)
    
    # Source environment
    source_environment()
    
    # Launch simulation
    success = launch_simulation()
    
    if not success:
        print("Failed to launch simulation")
        sys.exit(1)

if __name__ == "__main__":
    main() 