#!/usr/bin/env python3

import os
import sys
import json
import subprocess
import socket
from typing import Tuple, List

class VRSetupVerifier:
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
            
    def verify_android_tools(self):
        self.print_header("Checking Android Development Tools")
        
        # Check ADB
        adb_ok, adb_version = self.check_command("adb version")
        self.print_result("ADB installation", adb_ok, 
                         adb_version.split('\n')[0] if adb_ok else "Not found")
        
        # Check Java
        java_ok, java_version = self.check_command("java -version")
        self.print_result("Java installation", java_ok,
                         java_version.split('\n')[0] if java_ok else "Not found")
        
        # Check Android SDK
        sdk_path = os.environ.get("ANDROID_HOME")
        self.print_result("Android SDK path", bool(sdk_path), 
                         sdk_path if sdk_path else "Not set")
                         
    def verify_unity_installation(self):
        self.print_header("Checking Unity Installation")
        
        # Check Unity Hub
        unity_hub_ok, _ = self.check_command("which unityhub")
        self.print_result("Unity Hub", unity_hub_ok)
        
        # Check Unity project
        unity_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "unity")
        project_settings = os.path.join(unity_dir, "ProjectSettings")
        self.print_result("Unity project", os.path.isdir(project_settings))
        
        # Check Unity packages
        packages_file = os.path.join(unity_dir, "Packages", "manifest.json")
        if os.path.exists(packages_file):
            with open(packages_file) as f:
                packages = json.load(f)
                required_packages = {
                    "com.unity.xr.interaction.toolkit": "XR Interaction Toolkit",
                    "com.unity.xr.management": "XR Plugin Management",
                    "com.unity.xr.oculus": "Oculus XR Plugin",
                    "com.unity.robotics.ros-tcp-connector": "ROS TCP Connector"
                }
                for package, name in required_packages.items():
                    self.print_result(f"Package: {name}", 
                                    package in packages.get("dependencies", {}))
        else:
            self.print_result("Unity packages", False, "manifest.json not found")
            
    def verify_quest_connection(self):
        self.print_header("Checking Quest Connection")
        
        # Check ADB devices
        try:
            devices = subprocess.check_output(["adb", "devices"]).decode()
            devices = [line.split()[0] for line in devices.split('\n')[1:] if line.strip()]
            self.print_result("Quest connection", bool(devices),
                            f"Found {len(devices)} device(s)" if devices else "No devices")
        except:
            self.print_result("Quest connection", False, "ADB error")
            
    def verify_network(self):
        self.print_header("Checking Network Configuration")
        
        # Check ROS network settings
        ros_config = os.path.join(os.path.dirname(os.path.dirname(__file__)), 
                                "config", "ros_config.yaml")
        if os.path.exists(ros_config):
            self.print_result("ROS config", True, ros_config)
        else:
            self.print_result("ROS config", False, "Config file not found")
            
        # Check Unity network settings
        unity_config = os.path.join(os.path.dirname(os.path.dirname(__file__)), 
                                  "config", "unity_config.json")
        if os.path.exists(unity_config):
            with open(unity_config) as f:
                config = json.load(f)
                ip = config.get("ros_ip", "")
                port = config.get("ros_port", "")
                self.print_result("Unity network config", bool(ip and port),
                                f"IP: {ip}, Port: {port}")
                
                # Test network connection
                try:
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock.settimeout(1)
                    result = sock.connect_ex((ip, int(port)))
                    sock.close()
                    self.print_result("Network connection", result == 0,
                                    "Port open" if result == 0 else "Port closed")
                except:
                    self.print_result("Network connection", False, "Connection error")
        else:
            self.print_result("Unity network config", False, "Config file not found")
            
    def verify_ros_topics(self):
        self.print_header("Checking ROS Topics")
        
        # Check if ROS is running
        try:
            topics = subprocess.check_output(["ros2", "topic", "list"]).decode()
            topics = [t.strip() for t in topics.split('\n') if t.strip()]
            
            vr_topics = [t for t in topics if 'vr' in t.lower()]
            self.print_result("VR topics", bool(vr_topics),
                            f"Found {len(vr_topics)} VR topics" if vr_topics else "No VR topics")
            
            if vr_topics:
                for topic in vr_topics[:3]:  # Show first 3 topics
                    self.print_result(f"Topic: {topic}", True)
        except:
            self.print_result("ROS topics", False, "ROS 2 not running")
            
    def verify_all(self):
        """Run all verification checks"""
        self.verify_android_tools()
        self.verify_unity_installation()
        self.verify_quest_connection()
        self.verify_network()
        self.verify_ros_topics()
        
        self.print_header("Verification Summary")
        if self.all_passed:
            print("\033[92mAll VR setup checks passed! The system is ready for VR teleoperation.\033[0m")
        else:
            print("\033[91mSome checks failed. Please review the output above.\033[0m")
            if self.warnings:
                print("\nWarnings:")
                for warning in self.warnings:
                    print(f"- {warning}")
                    
        return self.all_passed

if __name__ == "__main__":
    verifier = VRSetupVerifier()
    success = verifier.verify_all()
    sys.exit(0 if success else 1) 