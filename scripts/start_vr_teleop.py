#!/usr/bin/env python3

import subprocess
import sys
import os
import signal
import time
import threading
from pathlib import Path
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition

def signal_handler(sig, frame):
    """Handle Ctrl+C to gracefully shutdown"""
    print("\nShutting down VR teleoperation...")
    sys.exit(0)

def check_vr_devices():
    """Check for VR devices"""
    print("Checking VR devices...")
    
    vr_devices = []
    
    # Check for Oculus Quest
    if Path("/dev/hidraw0").exists():
        vr_devices.append("Oculus Quest")
    
    # Check for SteamVR
    if shutil.which("vrcmd"):
        vr_devices.append("SteamVR")
    
    # Check for VR controllers
    if Path("/dev/input/js0").exists():
        vr_devices.append("VR Controllers")
    
    if vr_devices:
        print("VR devices found:")
        for device in vr_devices:
            print(f"  ✓ {device}")
        return True
    else:
        print("No VR devices detected")
        print("Please connect your VR headset and controllers")
        return False

def check_dependencies():
    """Check if required dependencies are available"""
    print("Checking dependencies...")
    
    checks = [
        ("ROS 2", "ros2 --version"),
        ("Unity", "unity-hub --version"),
        ("VR Runtime", "which vrcmd || which oculus-runtime")
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

def launch_unity_vr():
    """Launch Unity VR application"""
    print("Launching Unity VR application...")
    
    # Get Unity project path
    script_dir = Path(__file__).parent.parent
    unity_project = script_dir / "unity" / "UnityProject"
    
    if not unity_project.exists():
        print("Unity project not found. Please set up Unity integration first.")
        return False
    
    # Launch Unity with VR support
    command = f"""
    cd {unity_project} && \
    unity-hub -- --projectPath {unity_project} --headless --batchmode --executeMethod VRTeleop.StartVR
    """
    
    try:
        subprocess.Popen(command, shell=True)
        return True
    except Exception as e:
        print(f"Error launching Unity: {e}")
        return False

def launch_ros_nodes():
    """Launch ROS nodes for VR teleoperation"""
    print("Launching ROS nodes...")
    
    workspace_path = os.path.expanduser("~/ros_ws")
    
    # Launch cartesian commander
    commander_command = f"""
    cd {workspace_path} && \
    source install/setup.bash && \
    ros2 run so101_teleop_vr cartesian_commander
    """
    
    # Launch data recorder
    recorder_command = f"""
    cd {workspace_path} && \
    source install/setup.bash && \
    ros2 run so101_teleop_vr data_recorder
    """
    
    try:
        # Start commander in background
        commander_process = subprocess.Popen(commander_command, shell=True)
        
        # Start recorder in background
        recorder_process = subprocess.Popen(recorder_command, shell=True)
        
        return commander_process, recorder_process
    except Exception as e:
        print(f"Error launching ROS nodes: {e}")
        return None, None

def launch_vr_runtime():
    """Launch VR runtime"""
    print("Launching VR runtime...")
    
    # Try to launch SteamVR
    try:
        subprocess.Popen("vrcmd", shell=True)
        print("SteamVR launched")
        return True
    except:
        pass
    
    # Try to launch Oculus runtime
    try:
        subprocess.Popen("oculus-runtime", shell=True)
        print("Oculus runtime launched")
        return True
    except:
        pass
    
    print("Could not launch VR runtime automatically")
    print("Please launch your VR software manually")
    return False

def monitor_processes(processes):
    """Monitor running processes"""
    while True:
        for name, process in processes.items():
            if process.poll() is not None:
                print(f"{name} process terminated")
                return False
        time.sleep(1)

def generate_launch_description():
    # Declare the camera_type argument
    camera_type_arg = DeclareLaunchArgument(
        'camera_type',
        default_value='usb',
        description='Type of camera to use (usb or realsense)'
    )

    # Include the camera launch file
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('so101_camera'), 'launch', 'camera.launch.py')
        ),
        launch_arguments={'camera_type': LaunchConfiguration('camera_type')}.items()
    )

    camera_type = LaunchConfiguration('camera_type')
    
    launch_entities = [
        camera_type_arg,
    ]

    # Only include camera launch if not in sim mode
    camera_launch_condition = UnlessCondition(camera_type == 'sim')

    launch_entities.append(
        camera_launch if camera_launch.condition is None else camera_launch
    )
    # Add teleop nodes...
    launch_entities.extend([
        cartesian_commander_node,
        data_recorder_node,
    ])

    return LaunchDescription(launch_entities)

def main():
    """Main function"""
    print("SO-101 VR Teleoperation Launcher")
    print("=" * 35)
    
    # Set up signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Check VR devices
    if not check_vr_devices():
        print("VR devices not found. Please connect your VR headset.")
        response = input("Continue anyway? (y/N): ")
        if response.lower() != 'y':
            sys.exit(1)
    
    # Check dependencies
    if not check_dependencies():
        print("Dependency check failed. Please install missing dependencies.")
        sys.exit(1)
    
    # Launch VR runtime
    launch_vr_runtime()
    
    # Launch Unity VR
    if not launch_unity_vr():
        print("Failed to launch Unity VR")
        sys.exit(1)
    
    # Wait for Unity to start
    print("Waiting for Unity to start...")
    time.sleep(5)
    
    # Launch ROS nodes
    commander_process, recorder_process = launch_ros_nodes()
    
    if not commander_process or not recorder_process:
        print("Failed to launch ROS nodes")
        sys.exit(1)
    
    # Monitor processes
    processes = {
        "Commander": commander_process,
        "Recorder": recorder_process
    }
    
    print("VR teleoperation started successfully!")
    print("Press Ctrl+C to stop")
    
    try:
        monitor_processes(processes)
    except KeyboardInterrupt:
        print("\nShutting down VR teleoperation...")
        
        # Terminate processes
        for name, process in processes.items():
            if process.poll() is None:
                process.terminate()
                print(f"Terminated {name} process")

if __name__ == "__main__":
    main() 