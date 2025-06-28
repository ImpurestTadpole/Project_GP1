#!/usr/bin/env python3

import os
import sys
import subprocess
import shutil
from pathlib import Path
import git

def run_command(command, check=True, shell=True, cwd=None):
    """Run a shell command"""
    print(f"Running: {command}")
    try:
        result = subprocess.run(command, shell=shell, check=check, 
                              capture_output=True, text=True, cwd=cwd)
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

def clone_repositories():
    """Clone required repositories"""
    print("Cloning repositories...")
    
    # Get current script directory
    script_dir = Path(__file__).parent.parent
    workspace_path = Path.home() / "ros_ws" / "src"
    workspace_path.mkdir(parents=True, exist_ok=True)
    
    # Repositories to clone
    repositories = [
        {
            "url": "https://github.com/huggingface/lerobot.git",
            "path": workspace_path / "lerobot",
            "branch": "main"
        },
        {
            "url": "https://github.com/ros-controls/ros2_controllers.git",
            "path": workspace_path / "ros2_controllers",
            "branch": "humble"
        },
        {
            "url": "https://github.com/ros-controls/ros2_control.git",
            "path": workspace_path / "ros2_control",
            "branch": "humble"
        }
    ]
    
    for repo in repositories:
        if not repo["path"].exists():
            print(f"Cloning {repo['url']} to {repo['path']}")
            try:
                git.Repo.clone_from(repo["url"], repo["path"], branch=repo["branch"])
            except Exception as e:
                print(f"Failed to clone {repo['url']}: {e}")
        else:
            print(f"Repository already exists: {repo['path']}")

def copy_project_files():
    """Copy project files to workspace"""
    print("Copying project files...")
    
    script_dir = Path(__file__).parent.parent
    workspace_path = Path.home() / "ros_ws" / "src"
    
    # Copy ROS packages
    packages = ["so101_sim", "so101_teleop_vr", "lerobot", "pointcloud_mapper"]
    
    for package in packages:
        src_path = script_dir / "ros_ws" / "src" / package
        dst_path = workspace_path / package
        
        if src_path.exists():
            if dst_path.exists():
                shutil.rmtree(dst_path)
            shutil.copytree(src_path, dst_path)
            print(f"Copied {package} to workspace")
        else:
            print(f"Warning: {package} source not found at {src_path}")

def build_workspace():
    """Build the ROS workspace"""
    print("Building workspace...")
    
    workspace_path = Path.home() / "ros_ws"
    
    # Source ROS 2 and build
    commands = [
        "source /opt/ros/humble/setup.bash",
        "colcon build --symlink-install"
    ]
    
    for command in commands:
        run_command(command, cwd=workspace_path)

def setup_unity_integration():
    """Setup Unity integration"""
    print("Setting up Unity integration...")
    
    script_dir = Path(__file__).parent.parent
    unity_dir = script_dir / "unity"
    
    if unity_dir.exists():
        print("Unity integration files found")
        print("To use Unity integration:")
        print("1. Open Unity Hub")
        print("2. Add the Unity project from the unity/ directory")
        print("3. Install ROS-TCP-Connector package")
        print("4. Configure the ROS connection in Unity")
    else:
        print("Unity integration files not found")

def setup_vr_integration():
    """Setup VR integration"""
    print("Setting up VR integration...")
    
    # Check for VR devices
    vr_devices = []
    
    # Check for Oculus Quest
    if Path("/dev/hidraw0").exists():
        vr_devices.append("Oculus Quest detected")
    
    # Check for SteamVR
    if shutil.which("vrcmd"):
        vr_devices.append("SteamVR detected")
    
    if vr_devices:
        print("VR devices found:")
        for device in vr_devices:
            print(f"  - {device}")
    else:
        print("No VR devices detected")
        print("To use VR teleoperation:")
        print("1. Connect your VR headset")
        print("2. Install SteamVR or Oculus software")
        print("3. Run the VR teleoperation script")

def create_launch_scripts():
    """Create launch scripts"""
    print("Creating launch scripts...")
    
    script_dir = Path(__file__).parent.parent
    scripts_dir = script_dir / "scripts"
    
    # Create launch script for simulation
    sim_script = scripts_dir / "start_simulation.py"
    sim_content = '''#!/usr/bin/env python3

import subprocess
import sys
import os

def main():
    """Start simulation"""
    workspace_path = os.path.expanduser("~/ros_ws")
    
    # Source ROS and launch simulation
    command = f"cd {workspace_path} && source install/setup.bash && ros2 launch so101_sim sim_launch.py"
    
    subprocess.run(command, shell=True)

if __name__ == "__main__":
    main()
'''
    
    with open(sim_script, "w") as f:
        f.write(sim_content)
    
    # Make executable
    os.chmod(sim_script, 0o755)
    
    print("Created launch scripts")

def setup_data_directories():
    """Setup data directories"""
    print("Setting up data directories...")
    
    data_dirs = [
        "~/data/sim_episodes",
        "~/data/real_episodes", 
        "~/data/lerobot_episodes",
        "~/models/lerobot",
        "~/logs/lerobot",
        "~/checkpoints/lerobot"
    ]
    
    for dir_path in data_dirs:
        path = Path(dir_path).expanduser()
        path.mkdir(parents=True, exist_ok=True)
        print(f"Created directory: {path}")

def verify_installation():
    """Verify installation"""
    print("Verifying installation...")
    
    checks = [
        ("ROS 2", "ros2 --version"),
        ("Gazebo", "gazebo --version"),
        ("MoveIt", "ros2 pkg list | grep moveit"),
        ("LeRobot", "python3 -c 'import lerobot; print(\"LeRobot available\")'"),
        ("PyTorch", "python3 -c 'import torch; print(f\"PyTorch {torch.__version__}\")'"),
        ("Workspace", "ls ~/ros_ws/src")
    ]
    
    for name, command in checks:
        try:
            result = subprocess.run(command, shell=True, capture_output=True, text=True)
            if result.returncode == 0:
                print(f"✓ {name}: OK")
            else:
                print(f"✗ {name}: Failed")
        except Exception as e:
            print(f"✗ {name}: Error - {e}")

def main():
    """Main setup function"""
    print("SO-101 Workspace Setup")
    print("=" * 30)
    
    # Clone repositories
    clone_repositories()
    
    # Copy project files
    copy_project_files()
    
    # Build workspace
    build_workspace()
    
    # Setup integrations
    setup_unity_integration()
    setup_vr_integration()
    
    # Create launch scripts
    create_launch_scripts()
    
    # Setup data directories
    setup_data_directories()
    
    # Verify installation
    verify_installation()
    
    print("\nWorkspace setup complete!")
    print("\nNext steps:")
    print("1. Source the workspace: source ~/ros_ws/install/setup.bash")
    print("2. Start simulation: python3 scripts/start_simulation.py")
    print("3. Start VR teleoperation: python3 scripts/start_vr_teleop.py")
    print("4. Start training: python3 scripts/start_training.py")

if __name__ == "__main__":
    main() 