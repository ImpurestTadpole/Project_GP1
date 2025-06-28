#!/usr/bin/env python3

import os
import sys
import subprocess
import pkg_resources
import importlib
from typing import List, Tuple

class InstallationVerifier:
    def __init__(self):
        self.all_passed = True
        self.warnings = []
        
    def print_header(self, text: str):
        print("\n" + "=" * 80)
        print(f" {text}")
        print("=" * 80)
        
    def print_result(self, component: str, passed: bool, message: str = ""):
        status = "\033[92m✓\033[0m" if passed else "\033[91m✗\033[0m"
        print(f"{status} {component:<30} {message}")
        self.all_passed = self.all_passed and passed
        
    def check_command(self, command: str) -> Tuple[bool, str]:
        try:
            output = subprocess.check_output(command.split(), stderr=subprocess.STDOUT).decode()
            return True, output.strip()
        except:
            return False, ""
            
    def check_python_package(self, package: str) -> bool:
        try:
            importlib.import_module(package)
            return True
        except ImportError:
            return False
            
    def verify_ros2(self):
        self.print_header("Checking ROS 2 Installation")
        
        # Check ROS 2 environment
        ros_distro = os.environ.get("ROS_DISTRO")
        self.print_result("ROS_DISTRO", ros_distro == "humble", 
                         f"Found: {ros_distro or 'Not set'}")
        
        # Check core ROS 2 commands
        ros2_ok, version = self.check_command("ros2 --version")
        self.print_result("ros2 command", ros2_ok, version)
        
        # Check essential ROS 2 packages
        packages = ["rclpy", "geometry_msgs", "sensor_msgs", "cv_bridge"]
        for pkg in packages:
            self.print_result(f"ROS package: {pkg}", 
                            self.check_python_package(pkg))
                            
    def verify_python(self):
        self.print_header("Checking Python Environment")
        
        # Check Python version
        python_version = sys.version.split()[0]
        self.print_result("Python version", python_version.startswith("3."),
                         f"Version: {python_version}")
        
        # Check virtual environment
        in_venv = sys.prefix != sys.base_prefix
        self.print_result("Virtual environment", in_venv,
                         "Active" if in_venv else "Not active")
        
        # Check required packages
        required_packages = [
            "numpy",
            "torch",
            "stable_baselines3",
            "gymnasium",
            "opencv-python",
            "pyrealsense2",
            "transforms3d",
            "yaml"
        ]
        
        for package in required_packages:
            self.print_result(f"Package: {package}",
                            self.check_python_package(package.replace("-", "_")))
                            
    def verify_nvidia(self):
        self.print_header("Checking NVIDIA Setup")
        
        # Check NVIDIA driver
        nvidia_ok, nvidia_info = self.check_command("nvidia-smi")
        self.print_result("NVIDIA driver", nvidia_ok,
                         "GPU detected" if nvidia_ok else "Not found")
        
        # Check CUDA
        cuda_ok = False
        if self.check_python_package("torch"):
            import torch
            cuda_ok = torch.cuda.is_available()
        self.print_result("CUDA available", cuda_ok,
                         "Yes" if cuda_ok else "No")
                         
    def verify_workspace(self):
        self.print_header("Checking Workspace Setup")
        
        # Check ROS workspace
        ros_ws = os.path.expanduser("~/ros_ws")
        ws_exists = os.path.isdir(ros_ws)
        self.print_result("ROS workspace", ws_exists,
                         ros_ws if ws_exists else "Not found")
        
        # Check data directories
        data_dirs = [
            "~/data/sim_episodes",
            "~/data/real_episodes",
            "~/data/lerobot_episodes",
            "~/models/lerobot",
            "~/logs/lerobot",
            "~/checkpoints/lerobot"
        ]
        
        for dir_path in data_dirs:
            expanded_path = os.path.expanduser(dir_path)
            self.print_result(f"Directory: {dir_path}", 
                            os.path.isdir(expanded_path))
                            
    def verify_gazebo(self):
        self.print_header("Checking Gazebo Installation")
        
        # Check Gazebo
        gazebo_ok, version = self.check_command("gazebo --version")
        self.print_result("Gazebo", gazebo_ok, 
                         version if gazebo_ok else "Not found")
        
        # Check Gazebo ROS packages
        gz_packages = [
            "gazebo_ros",
            "gazebo_ros2_control",
            "gazebo_plugins"
        ]
        
        for pkg in gz_packages:
            ok, _ = self.check_command(f"ros2 pkg list | grep {pkg}")
            self.print_result(f"Package: {pkg}", ok)
            
    def verify_all(self):
        """Run all verification checks"""
        self.verify_ros2()
        self.verify_python()
        self.verify_nvidia()
        self.verify_workspace()
        self.verify_gazebo()
        
        self.print_header("Verification Summary")
        if self.all_passed:
            print("\033[92mAll checks passed! The system is ready to use.\033[0m")
        else:
            print("\033[91mSome checks failed. Please review the output above.\033[0m")
            if self.warnings:
                print("\nWarnings:")
                for warning in self.warnings:
                    print(f"- {warning}")
                    
        return self.all_passed

if __name__ == "__main__":
    verifier = InstallationVerifier()
    success = verifier.verify_all()
    sys.exit(0 if success else 1) 