#!/usr/bin/env python3

import subprocess
import sys
import os
import signal
import time
import argparse
import threading
from pathlib import Path

def signal_handler(sig, frame):
    """Handle Ctrl+C to gracefully shutdown"""
    print("\nShutting down full pipeline...")
    sys.exit(0)

def check_system_status():
    """Check if all system components are ready"""
    print("Checking system status...")
    
    checks = [
        ("ROS 2", "ros2 --version"),
        ("Gazebo", "gazebo --version"),
        ("LeRobot", "python3 -c 'import lerobot; print(\"LeRobot available\")'"),
        ("Unity", "which unity-hub"),
        ("VR Runtime", "which vrcmd || which oculus-runtime"),
        ("Workspace", "ls ~/ros_ws/src/so101_sim")
    ]
    
    all_ready = True
    for name, command in checks:
        try:
            result = subprocess.run(command, shell=True, capture_output=True, text=True)
            if result.returncode == 0:
                print(f"✓ {name}: Ready")
            else:
                print(f"✗ {name}: Not ready")
                all_ready = False
        except Exception as e:
            print(f"✗ {name}: Error - {e}")
            all_ready = False
    
    return all_ready

def launch_component(component_name, command, cwd=None):
    """Launch a system component"""
    print(f"Launching {component_name}...")
    
    try:
        process = subprocess.Popen(
            command, 
            shell=True, 
            cwd=cwd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        return process
    except Exception as e:
        print(f"Error launching {component_name}: {e}")
        return None

def monitor_processes(processes):
    """Monitor running processes"""
    while True:
        for name, process in processes.items():
            if process and process.poll() is not None:
                print(f"{name} process terminated with code {process.returncode}")
                return False
        time.sleep(1)

def launch_simulation_pipeline():
    """Launch simulation components"""
    workspace_path = os.path.expanduser("~/ros_ws")
    
    # Launch Gazebo simulation
    sim_command = f"""
    cd {workspace_path} && \
    source install/setup.bash && \
    ros2 launch so101_sim sim_launch.py use_sim_time:=true gui:=true rviz:=true
    """
    
    sim_process = launch_component("Simulation", sim_command)
    
    # Wait for simulation to start
    time.sleep(5)
    
    return sim_process

def launch_vr_pipeline():
    """Launch VR teleoperation components"""
    workspace_path = os.path.expanduser("~/ros_ws")
    
    # Launch cartesian commander
    commander_command = f"""
    cd {workspace_path} && \
    source install/setup.bash && \
    ros2 run so101_teleop_vr cartesian_commander
    """
    
    commander_process = launch_component("VR Commander", commander_command)
    
    # Launch data recorder
    recorder_command = f"""
    cd {workspace_path} && \
    source install/setup.bash && \
    ros2 run so101_teleop_vr data_recorder
    """
    
    recorder_process = launch_component("Data Recorder", recorder_command)
    
    return commander_process, recorder_process

def launch_training_pipeline():
    """Launch training components"""
    workspace_path = os.path.expanduser("~/ros_ws")
    
    # Launch LeRobot trainer
    trainer_command = f"""
    cd {workspace_path} && \
    source install/setup.bash && \
    python3 src/lerobot/lerobot/train.py
    """
    
    trainer_process = launch_component("LeRobot Trainer", trainer_command)
    
    return trainer_process

def launch_unity_vr():
    """Launch Unity VR application"""
    script_dir = Path(__file__).parent.parent
    unity_project = script_dir / "unity" / "UnityProject"
    
    if not unity_project.exists():
        print("Unity project not found. Skipping Unity launch.")
        return None
    
    unity_command = f"""
    cd {unity_project} && \
    unity-hub -- --projectPath {unity_project} --headless --batchmode --executeMethod VRTeleop.StartVR
    """
    
    unity_process = launch_component("Unity VR", unity_command)
    
    return unity_process

def launch_vr_runtime():
    """Launch VR runtime"""
    print("Launching VR runtime...")
    
    # Try SteamVR
    try:
        subprocess.Popen("vrcmd", shell=True)
        print("SteamVR launched")
        return True
    except:
        pass
    
    # Try Oculus runtime
    try:
        subprocess.Popen("oculus-runtime", shell=True)
        print("Oculus runtime launched")
        return True
    except:
        pass
    
    print("Could not launch VR runtime automatically")
    return False

def main():
    """Main function"""
    parser = argparse.ArgumentParser(description='SO-101 Full Pipeline Launcher')
    parser.add_argument('--mode', choices=['sim', 'vr', 'training', 'full'], 
                       default='full', help='Pipeline mode')
    parser.add_argument('--skip-checks', action='store_true', 
                       help='Skip system checks')
    parser.add_argument('--no-vr', action='store_true', 
                       help='Skip VR components')
    parser.add_argument('--headless', action='store_true', 
                       help='Run in headless mode')
    
    args = parser.parse_args()
    
    print("SO-101 Full Pipeline Launcher")
    print("=" * 35)
    
    # Set up signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Check system status
    if not args.skip_checks and not check_system_status():
        print("System check failed. Please fix issues before continuing.")
        sys.exit(1)
    
    # Initialize processes dictionary
    processes = {}
    
    try:
        # Launch based on mode
        if args.mode in ['sim', 'full']:
            print("\n=== Launching Simulation Pipeline ===")
            sim_process = launch_simulation_pipeline()
            if sim_process:
                processes["Simulation"] = sim_process
            
            # Wait for simulation to stabilize
            time.sleep(10)
        
        if args.mode in ['vr', 'full'] and not args.no_vr:
            print("\n=== Launching VR Pipeline ===")
            
            # Launch VR runtime
            launch_vr_runtime()
            
            # Launch Unity VR
            unity_process = launch_unity_vr()
            if unity_process:
                processes["Unity VR"] = unity_process
            
            # Wait for Unity to start
            time.sleep(5)
            
            # Launch VR components
            commander_process, recorder_process = launch_vr_pipeline()
            if commander_process:
                processes["VR Commander"] = commander_process
            if recorder_process:
                processes["Data Recorder"] = recorder_process
        
        if args.mode in ['training', 'full']:
            print("\n=== Launching Training Pipeline ===")
            trainer_process = launch_training_pipeline()
            if trainer_process:
                processes["LeRobot Trainer"] = trainer_process
        
        # Monitor all processes
        print(f"\n=== Pipeline Status ===")
        print(f"Active components: {list(processes.keys())}")
        print("Press Ctrl+C to stop all components")
        
        monitor_processes(processes)
        
    except KeyboardInterrupt:
        print("\nShutting down pipeline...")
        
        # Terminate all processes
        for name, process in processes.items():
            if process and process.poll() is None:
                print(f"Terminating {name}...")
                process.terminate()
                
                # Wait for graceful shutdown
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    print(f"Force killing {name}...")
                    process.kill()
    
    except Exception as e:
        print(f"Error in pipeline: {e}")
        
        # Cleanup on error
        for name, process in processes.items():
            if process and process.poll() is None:
                process.terminate()
    
    print("Pipeline shutdown complete.")

if __name__ == "__main__":
    main() 