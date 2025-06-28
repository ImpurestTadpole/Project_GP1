#!/usr/bin/env python3

import os
import sys
import subprocess
import platform
import shutil
from pathlib import Path

def run_command(command, check=True, shell=True):
    """Run a shell command"""
    print(f"Running: {command}")
    try:
        result = subprocess.run(command, shell=shell, check=check, 
                              capture_output=True, text=True)
        if result.stdout:
            print(result.stdout)
        return result
    except subprocess.CalledProcessError as e:
        print(f"Error running command: {e}")
        if e.stderr:
            print(f"Error output: {e.stderr}")
        if check:
            sys.exit(1)
        return e

def check_system():
    """Check system requirements"""
    print("Checking system requirements...")
    
    # Check OS
    if platform.system() != "Linux":
        print("Error: This script is designed for Ubuntu Linux")
        sys.exit(1)
    
    # Check Ubuntu version
    try:
        with open("/etc/os-release", "r") as f:
            content = f.read()
            if "Ubuntu" not in content:
                print("Error: This script is designed for Ubuntu")
                sys.exit(1)
    except FileNotFoundError:
        print("Warning: Could not determine OS version")
    
    # Check if running as root
    if os.geteuid() == 0:
        print("Error: Do not run this script as root")
        sys.exit(1)
    
    print("System check passed")

def update_system():
    """Update system packages"""
    print("Updating system packages...")
    
    commands = [
        "sudo apt update",
        "sudo apt upgrade -y",
        "sudo apt autoremove -y"
    ]
    
    for command in commands:
        run_command(command)

def install_ros2():
    """Install ROS 2 Humble"""
    print("Installing ROS 2 Humble...")
    
    # Add ROS 2 repository
    commands = [
        "sudo apt update && sudo apt install software-properties-common -y",
        "sudo add-apt-repository universe -y",
        "sudo apt update && sudo apt install curl -y",
        "sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg",
        "echo 'deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main' | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null",
        "sudo apt update",
        "sudo apt install ros-humble-desktop -y",
        "sudo apt install ros-dev-tools -y"
    ]
    
    for command in commands:
        run_command(command)

def install_gazebo():
    """Install Gazebo"""
    print("Installing Gazebo...")
    
    commands = [
        "sudo apt install gazebo -y",
        "sudo apt install libgazebo-dev -y",
        "sudo apt install ros-humble-gazebo-ros-pkgs -y",
        "sudo apt install ros-humble-gazebo-ros2-control -y"
    ]
    
    for command in commands:
        run_command(command)

def install_moveit():
    """Install MoveIt"""
    print("Installing MoveIt...")
    
    commands = [
        "sudo apt install ros-humble-moveit -y",
        "sudo apt install ros-humble-moveit-ros-planning-interface -y",
        "sudo apt install ros-humble-moveit-ros-move-group -y",
        "sudo apt install ros-humble-moveit-ros-control-interface -y"
    ]
    
    for command in commands:
        run_command(command)

def install_python_deps():
    """Install Python dependencies"""
    print("Installing Python dependencies...")
    
    # Check if pip is available
    if shutil.which("pip3") is None:
        run_command("sudo apt install python3-pip -y")
    
    # Install Python packages
    packages = [
        "numpy",
        "opencv-python",
        "matplotlib",
        "scipy",
        "scikit-learn",
        "torch",
        "torchvision",
        "transformers",
        "stable-baselines3",
        "gymnasium",
        "pyyaml",
        "rospkg",
        "catkin-tools",
        "colcon-common-extensions"
    ]
    
    for package in packages:
        run_command(f"pip3 install {package}")

def install_lerobot():
    """Install LeRobot"""
    print("Installing LeRobot...")
    
    # Check for NVIDIA GPU
    nvidia_gpu = False
    try:
        result = subprocess.run("nvidia-smi", shell=True, capture_output=True)
        if result.returncode == 0:
            nvidia_gpu = True
            print("NVIDIA GPU detected")
    except:
        pass
    
    # Install CUDA if NVIDIA GPU is available
    if nvidia_gpu:
        print("Installing CUDA support...")
        commands = [
            "sudo apt install nvidia-cuda-toolkit -y",
            "pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu118"
        ]
        for command in commands:
            run_command(command)
    else:
        print("No NVIDIA GPU detected, installing CPU-only PyTorch")
        run_command("pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cpu")
    
    # Install LeRobot
    run_command("pip3 install lerobot")

def setup_workspace():
    """Setup ROS workspace"""
    print("Setting up ROS workspace...")
    
    # Create workspace directory
    workspace_path = Path.home() / "ros_ws"
    workspace_path.mkdir(exist_ok=True)
    
    # Source ROS 2
    ros_source = "source /opt/ros/humble/setup.bash"
    
    # Initialize workspace
    commands = [
        f"cd {workspace_path}",
        f"{ros_source} && colcon build"
    ]
    
    for command in commands:
        run_command(command)

def setup_environment():
    """Setup environment variables"""
    print("Setting up environment...")
    
    # Add to .bashrc
    bashrc_path = Path.home() / ".bashrc"
    
    environment_lines = [
        "",
        "# ROS 2 Setup",
        "source /opt/ros/humble/setup.bash",
        "source ~/ros_ws/install/setup.bash",
        "",
        "# Gazebo Setup",
        "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros_ws/src/so101_sim/models",
        "export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/ros_ws/src/so101_sim/worlds",
        "",
        "# Python path",
        "export PYTHONPATH=$PYTHONPATH:~/ros_ws/src"
    ]
    
    # Check if already added
    with open(bashrc_path, "r") as f:
        content = f.read()
    
    for line in environment_lines:
        if line not in content:
            with open(bashrc_path, "a") as f:
                f.write(line + "\n")
    
    print("Environment setup complete. Please restart your terminal or run 'source ~/.bashrc'")

def main():
    """Main installation function"""
    print("SO-101 Dual-Arm VR Teleoperation System Installation")
    print("=" * 50)
    
    # Check system
    check_system()
    
    # Update system
    update_system()
    
    # Install components
    install_ros2()
    install_gazebo()
    install_moveit()
    install_python_deps()
    install_lerobot()
    
    # Setup workspace
    setup_workspace()
    
    # Setup environment
    setup_environment()
    
    print("\nInstallation complete!")
    print("Next steps:")
    print("1. Restart your terminal or run 'source ~/.bashrc'")
    print("2. Build the ROS workspace: cd ~/ros_ws && colcon build")
    print("3. Run the setup script: python3 setup/setup_workspace.py")

if __name__ == "__main__":
    main() 