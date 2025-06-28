# General Robotic Arm Learning Pipeline
A complete, bootable Ubuntu system for dual-arm robot VR teleoperation, reinforcement learning, and policy deployment on both **simulated and real robots**.

## System Overview

This project provides a full pipeline for controlling a dual-arm robot using a VR headset, recording data, and training policies with [LeRobot](https://github.com/huggingface/lerobot). It features a clear separation between simulation and real hardware, allowing for seamless transition from training in sim to deploying on a real robot. The system is visualized with [RViz](https://github.com/ros2/rviz) and [Rerun](https://github.com/rerun-io/rerun).

## System Requirements
-   **OS**: Ubuntu 22.04 LTS (primary), Windows 10/11 (development)
-   **Hardware**: NVIDIA GPU (recommended), Meta Quest 3 or compatible VR headset, and a physical robot arm (for real-world operation)
-   **Dependencies**: ROS 2 Humble, Gazebo, Python 3.8+, PyTorch

---

## Step-by-Step Execution Guide

### Step 1: Installation
First, clone this repository to your Ubuntu machine. Then, run the main installation script to install all dependencies.
```bash
# Make the installer executable and run it
chmod +x install_all.sh
./install_all.sh

# Source your environment
source ~/.bashrc
```

**Troubleshooting Installation Issues:**
If you encounter Git repository errors during LeRobot installation, run the fix script:
```bash
chmod +x fix_lerobot_install.sh
./fix_lerobot_install.sh
```

### Step 2: Choose Your Mode (Sim or Real)

You can run the system in either **simulation** or with a **real robot**.

#### **Option A: Running in Simulation**
This option does not use a physical camera and relies on the simulated camera within Gazebo.
You will need **three separate terminals**.

**➡️ Terminal 1: Launch the Simulation**
```bash
# This starts Gazebo and RViz
python3 scripts/start_simulation.py
```

**➡️ Terminal 2: Launch VR Teleop & Data Recording**
```bash
# This starts the VR bridge and data recorder
python3 scripts/start_vr_teleop.py
```

**➡️ Terminal 3: Launch the Rerun Visualization**
```bash
# This opens the Rerun viewer for live data
python3 scripts/visualize_data.py
```
---
#### **Option B: Running on a Real Robot**

**Before you start:** 
- If using a real arm, you must integrate your robot's specific SDK into the `ros_ws/src/so101_bringup/so101_bringup/hardware_interface.py` file.
- This system supports both standard USB cameras and Intel RealSense D400 series cameras.

You will need **three separate terminals**.

**➡️ Terminal 1: Launch the Real Robot Bringup**
```bash
# This starts your robot's hardware interface and RViz
ros2 launch so101_bringup real.launch.py
```

**➡️ Terminal 2: Launch VR Teleop, Camera, & Data Recording**
Choose one of the following commands based on your camera hardware.

*To use a standard USB camera:*
```bash
# The 'usb' camera_type is the default
python3 scripts/start_vr_teleop.py
```

*To use an Intel RealSense camera:*
```bash
# Specify the 'realsense' camera_type
python3 scripts/start_vr_teleop.py camera_type:=realsense
```

**➡️ Terminal 3: Launch the Rerun Visualization**
```bash
# The Rerun viewer works for both sim and real
python3 scripts/visualize_data.py
```

### Step 3: The Data & Training Workflow

This workflow is the same for both sim and real.
1.  **Record Demonstrations**: Use the VR controls (**R** to start, **S** to stop) to record task demonstrations. Data is saved to `~/data/raw_episodes/`.
2.  **Convert Data**: Run `python3 scripts/convert_to_lerobot.py` to process the raw data into the LeRobot dataset format.
3.  **Train Policy**: Choose between three training approaches:
    - **Imitation Learning**: Run `python3 scripts/start_training.py` to train a policy on your dataset using supervised learning.
    - **Simple RL**: Run `python3 scripts/start_simple_rl_training.py` for basic reinforcement learning using stable-baselines3.
    - **HIL-SERL (Advanced RL)**: Run `python3 scripts/start_rl_training.py` for Human-in-the-Loop Sample-Efficient Reinforcement Learning (requires additional dependencies).

### Step 4: Deploying a Trained Policy

Once you have a trained policy, you can deploy it to control the robot autonomously in either sim or real mode.

1.  **Start the environment**: Launch either the **simulation** (Step 2, Option A, Terminal 1) or the **real robot** (Step 2, Option B, Terminal 1).
2.  **Deploy the policy** (in a new terminal):
    ```bash
    # For sim, no camera argument is needed
    python3 scripts/deploy_policy.py --policy-path path/to/your/policy
    
    # For a real robot with a USB camera
    python3 scripts/deploy_policy.py --policy-path path/to/your/policy --camera-type usb

    # For a real robot with a RealSense camera
    python3 scripts/deploy_policy.py --policy-path path/to/your/policy --camera-type realsense
    ```
The robot will now be controlled by your trained policy.

---
## VR Application Setup for Meta Quest

To use VR teleoperation, you need to build and install the Unity VR application on your Meta Quest headset.

### Prerequisites
- Meta Quest 3 or compatible VR headset
- Unity Hub and Unity Editor (2021.3 LTS or newer recommended)
- Android SDK and build tools
- USB cable for Quest connection
- Developer mode enabled on Quest

### Step 1: Enable Developer Mode on Quest

1. **Install Meta Quest Developer Hub** on your computer from [developer.oculus.com](https://developer.oculus.com)
2. **Create Meta Developer Account** if you don't have one
3. **Enable Developer Mode**:
   - Open the Meta Quest mobile app
   - Go to Settings → Developer Mode
   - Toggle "Developer Mode" to ON
   - Follow the prompts to verify your developer account

### Step 2: Install Unity and Required Packages

#### Option A: On Windows/Mac
1. **Install Unity Hub** from [unity.com](https://unity.com/download)
2. **Install Unity Editor 2021.3 LTS** (or newer) through Unity Hub
3. **Install Android Build Support**:
   - In Unity Hub, go to Installs
   - Click the gear icon next to your Unity version
   - Select "Add Modules"
   - Check "Android Build Support" and its sub-components

#### Option B: On Linux (Ubuntu)
1. **Install Unity Hub**:
   ```bash
   # Download Unity Hub for Linux
   wget -O UnityHub.AppImage https://public-cdn.cloud.unity3d.com/hub/prod/UnityHub.AppImage
   chmod +x UnityHub.AppImage
   ./UnityHub.AppImage
   ```

2. **Install Unity Editor**:
   ```bash
   # Alternative: Install Unity directly via snap
   sudo snap install unity --classic
   
   # Or download specific version manually
   wget -O unity-editor.tar.xz https://download.unity3d.com/download_unity/[hash]/LinuxEditorInstaller/Unity.tar.xz
   tar -xf unity-editor.tar.xz
   ```

3. **Install Android SDK and Build Tools**:
   ```bash
   # Install Android development tools
   sudo apt update
   sudo apt install -y openjdk-11-jdk android-sdk
   
   # Set JAVA_HOME
   export JAVA_HOME=/usr/lib/jvm/java-11-openjdk-amd64
   echo 'export JAVA_HOME=/usr/lib/jvm/java-11-openjdk-amd64' >> ~/.bashrc
   
   # Install Android SDK components
   sudo apt install -y android-sdk-platform-tools android-sdk-build-tools
   ```

4. **Configure Unity for Android**:
   - In Unity Hub or Unity Editor preferences
   - Set Android SDK path: `/usr/lib/android-sdk`
   - Set JDK path: `/usr/lib/jvm/java-11-openjdk-amd64`

### Step 3: Open and Configure Unity Project

1. **Open Unity Project**:
   ```bash
   # Launch Unity Hub and click "Open"
   # Navigate to: Project_GP1/unity/
   # Select the folder and click "Open"
   ```

2. **Install Required Packages**:
   - Open Window → Package Manager
   - Install these packages:
     - XR Interaction Toolkit (2.5.2+)
     - XR Plugin Management (4.4.0+)
     - Oculus XR Plugin (4.1.2+)
     - ROS TCP Connector (0.7.0+)

3. **Configure XR Settings**:
   - Go to Edit → Project Settings
   - Navigate to XR Plug-in Management
   - Check "Initialize XR on Startup"
   - Under "Plug-in Providers", enable "Oculus"

### Step 4: Configure Build Settings

1. **Switch to Android Platform**:
   - Go to File → Build Settings
   - Select "Android" and click "Switch Platform"

2. **Configure Android Settings**:
   - Click "Player Settings"
   - Set these values:
     - Company Name: `SO101`
     - Product Name: `VR Teleoperation`
     - Minimum API Level: `23`
     - Target API Level: `32`
     - Architecture: `ARM64`

3. **Configure XR Settings**:
   - In Player Settings, go to XR Settings
   - Check "Virtual Reality Supported"
   - Add "Oculus" to the list

### Step 5: Configure Network Settings

1. **Update ROS Connection**:
   - In the Unity scene, find the "ROS TCP Connector" GameObject
   - Set the ROS IP Address to your Ubuntu machine's IP:
     ```bash
     # On Ubuntu, find your IP with:
     ip addr show | grep "inet " | grep -v 127.0.0.1
     ```
   - Set ROS Port to `10000` (default)

2. **Update Unity Config**:
   - Edit `config/unity_config.json` if needed
   - Ensure the IP matches your Ubuntu machine

### Step 6: Build the APK

1. **Build the Application**:
   - Go to File → Build Settings
   - Click "Build"
   - Choose a location to save the APK (e.g., `builds/SO101-VR.apk`)
   - Wait for the build to complete (may take 5-15 minutes)

### Step 7: Install on Quest

1. **Connect Quest to Computer**:
   - Use USB cable to connect Quest to your computer
   - Put on the headset and allow USB debugging when prompted

2. **Install ADB Tools**:
   
   **On Ubuntu/Linux**:
   ```bash
   # Install Android Debug Bridge
   sudo apt update
   sudo apt install -y android-tools-adb android-tools-fastboot
   
   # Add user to plugdev group for USB access
   sudo usermod -a -G plugdev $USER
   # Log out and back in for group changes to take effect
   ```
   
   **On Windows**:
   ```bash
   # Download ADB from Android SDK or install via chocolatey
   choco install adb
   ```

3. **Verify Connection**:
   ```bash
   # Check if Quest is detected
   adb devices
   # Should show your Quest device like:
   # 1WMHH123456789  device
   ```

3. **Install the APK**:
   ```bash
   # Navigate to where you saved the APK
   cd builds/
   
   # Install the application
   adb install SO101-VR.apk
   
   # If updating an existing install, use:
   adb install -r SO101-VR.apk
   ```

### Step 8: Launch and Test

1. **Find the App on Quest**:
   - Put on your Quest headset
   - Go to App Library
   - Look under "Unknown Sources"
   - Find "VR Teleoperation" and launch it

2. **Test Connection**:
   ```bash
   # On Ubuntu, start the ROS bridge
   ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000
   
   # In another terminal, monitor VR data
   ros2 topic echo /so101_teleop/cmd_pose
   
   # Launch the VR app on Quest and move controllers
   # You should see pose data in the terminal
   ```

### Troubleshooting VR Setup

**Quest Not Detected by ADB**:
- Ensure Developer Mode is enabled
- Try different USB cable or port
- Check USB debugging permission on Quest

**Build Errors in Unity**:
- Verify all required packages are installed
- Check Android SDK path in Unity preferences
- Ensure target API levels are compatible

**No VR Tracking**:
- Check that Oculus XR Plugin is enabled
- Verify "Initialize XR on Startup" is checked
- Restart Unity and rebuild if needed

**ROS Connection Issues**:
- Verify IP addresses match between Unity and Ubuntu
- Check firewall settings on Ubuntu
- Ensure ROS TCP endpoint is running

Once the VR app is successfully installed and connected, you can proceed with the normal teleoperation workflow described in the previous sections.

### Linux Development Advantages

**Why develop VR on Linux:**
- **Same machine as ROS**: No network complexity between development and runtime
- **Faster iteration**: Build and test on the same system running the robot
- **Better integration**: Direct access to ROS topics and debugging tools
- **Performance**: Lower latency since Unity and ROS run on same machine

**Simplified workflow on Linux:**
```bash
# Everything on one machine - no IP configuration needed
# Terminal 1: Start ROS system
python3 scripts/start_simulation.py

# Terminal 2: Start ROS-Unity bridge  
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1

# Terminal 3: Open Unity and test in editor (no Quest needed for development)
./UnityHub.AppImage

# Terminal 4: Monitor data
ros2 topic echo /so101_teleop/cmd_pose
```

**Development tips:**
- Use Unity's "Play Mode" to test VR functionality without building APK
- Connect Quest for final testing only
- Use `localhost` (127.0.0.1) for ROS IP when developing on same machine
- Unity's console shows real-time ROS connection status

---
## HIL-SERL (Human-in-the-Loop Reinforcement Learning)

The system now supports advanced RL training using HIL-SERL, which combines human demonstrations with reinforcement learning for sample-efficient policy training.

### HIL-SERL Training Process

1. **Start with Demonstrations**: The system uses your VR demonstrations as a starting point
2. **Train Reward Classifier**: A neural network learns to predict rewards from human preferences
3. **RL Policy Training**: The policy is trained using RL with the learned reward function
4. **Human Interventions**: During training, you can intervene via VR to correct the robot's actions
5. **Continuous Improvement**: The system learns from both successes and interventions

### RL Training Options

**Simple RL (Recommended for beginners):**
```bash
# For simulation
python3 scripts/start_simple_rl_training.py --mode sim

# For real robot
python3 scripts/start_simple_rl_training.py --mode real
```

**Advanced HIL-SERL (Requires additional setup):**
```bash
# For simulation
python3 scripts/start_rl_training.py --mode sim

# For real robot
python3 scripts/start_rl_training.py --mode real
```

### VR Controller Controls During RL Training

- **A Button**: Start/Stop human intervention (take control of the robot)
- **B Button**: Give positive feedback (good action)
- **X Button**: Give negative feedback (bad action)
- **Controller Movement**: Direct robot control during interventions

### Configuration

RL training parameters can be configured in `config/rl_config.yaml`:
- Training hyperparameters (learning rates, batch sizes)
- Network architectures
- Intervention settings
- Reward classifier parameters

---
## Project Structure

The repository is organized to separate different components of the system, such as ROS packages, configuration files, and high-level scripts.

- **`/`**: Root directory containing the main installer (`install_all.sh`), documentation, and this `README`.
- **`config/`**: Contains all configuration files for the system.
  - `lerobot_config.yaml`: Configuration for LeRobot imitation learning training.
  - `rl_config.yaml`: Configuration for HIL-SERL reinforcement learning training.
  - `ros_config.yaml`: General ROS parameters.
  - `sim_params.yaml` / `real_params.yaml`: Parameters for simulation vs. real hardware.
  - `unity_config.json`: Configuration for the Unity VR application.
- **`docs/`**: Detailed setup and usage guides.
- **`ros_ws/`**: The main ROS 2 workspace.
  - `src/`: Contains all the ROS 2 packages for robot control, simulation, and teleoperation.
    - `so101_bringup`: Launches and configures the real robot hardware.
    - `so101_camera`: Handles camera drivers (USB, RealSense).
    - `so101_lerobot`: Integrates LeRobot with ROS 2 for policy deployment and HIL-SERL training.
    - `so101_sim`: Contains the simulation environment (URDF, worlds, launch files).
    - `so101_teleop_vr`: Manages VR teleoperation and data recording logic.
- **`scripts/`**: High-level Python scripts that act as the main entry points for users.
  - `start_simulation.py`: Launches the Gazebo simulation.
  - `start_vr_teleop.py`: Starts the VR teleoperation system.
  - `start_training.py`: Begins the LeRobot imitation learning training process.
  - `start_rl_training.py`: Launches HIL-SERL reinforcement learning training.
  - `deploy_policy.py`: Runs a trained policy on the robot.
  - `convert_to_lerobot.py`: Processes recorded data into the correct format for training.
- **`setup/`**: Helper scripts for installation and environment setup.
- **`unity/`**: Contains the Unity project for the VR interface.

## Teleoperation Modes

The multi-platform teleop layer lets you plug *any* input device into *any* robot. Pick a control backend with `--control` (default `vr`) and a robot type with `--robot` (default `so101`).

| Control Backend | CLI flag            | Typical device(s)                     |
|-----------------|---------------------|---------------------------------------|
| VR (default)    | `--control vr`      | Meta Quest 3, Varjo XR-4              |
| Keyboard        | `--control keyboard`| Laptop keyboard (W A S D + Q E)       |
| Gamepad         | `--control gamepad` | Xbox / PlayStation controller         |
| Leader Follower | `--control leader_follower` | Follows `/leader/*` ROS topics |

Example commands:
```bash
# 1) Drive a UR5 with the keyboard (no ROS needed):
python3 scripts/start_teleop.py --control keyboard --robot ur5

# 2) Xbox gamepad controlling a Franka arm *and* broadcasting as leader
python3 scripts/start_teleop.py --control gamepad --robot franka --broadcast-leader

# 3) Mirror the leader on a second robot (follower side)
python3 scripts/start_leader_follower_teleop.py --robot so101
```

Behind the scenes the script instantiates the correct `ControlInterface` (keyboard, gamepad, VR, …) and a `RobotInterface` (SO-101, UR5, Franka, …) via the factory helpers in `so101_lerobot/lerobot/factories.py`.

### Leader → Follower workflow

1. **Leader side** ‑ run *any* teleop session with the `--broadcast-leader` flag. This publishes the commanded pose and gripper fraction to ROS 2 topics `/leader/pose` and `/leader/gripper`.
2. **Follower side** ‑ start another robot with the `leader_follower` backend (shortcut script `scripts/start_leader_follower_teleop.py`). The follower subscribes to the topics and mirrors the motion in real-time.

You can chain an arbitrary number of followers (e.g. for classroom demonstrations) – just launch the follower script once per robot.

---
## Updated Project Structure (high-level)

```text
Project_GP1/
├── config/
│   └── robots/            # Per-robot YAML specs (UR5, Franka,…)
├── ros_ws/
│   └── src/
│       └── so101_lerobot/
│           └── lerobot/
│               ├── control/            # Input devices
│               │   ├── keyboard_control.py
│               │   ├── gamepad_control.py
│               │   ├── vr_control.py
│               │   └── leader_follower_control.py
│               ├── robots/             # Robot drivers / adapters
│               │   ├── so101_interface.py
│               │   ├── ur5_interface.py
│               │   └── franka_interface.py
│               ├── factories.py        # Dynamically builds the above
│               ├── control_interface.py
│               └── robot_interface.py
├── scripts/
│   ├── start_teleop.py                # Generic launcher (all controls)
│   └── start_leader_follower_teleop.py# Convenience wrapper for follower
└── docs/
    └── … (setup, usage, and workflow guides)
```

See the *docs* folder for an in-depth developer guide to each package.

---
### Optional Conda ML Environment

ROS 2 plays best with the **system Python**, but you may prefer a **separate Conda environment for training** to keep heavy ML libraries isolated.

1. Install [Miniconda](https://docs.conda.io/en/latest/miniconda.html) if you haven't already.
2. Run the helper script:
   ```bash
   bash setup/setup_conda_ml_env.sh     # creates env "lerobot"
   conda activate lerobot
   ```
3. Train as normal:
   ```bash
   python scripts/start_training.py  --dataset my_dataset  --config config/lerobot_config.yaml
   ```

You can leave ROS 2 nodes running in another terminal that's **not** inside the Conda env – only the training process needs it.

### LeRobot integration

The installation script clones and installs
[LeRobot](https://github.com/huggingface/lerobot) (Apache-2.0) automatically. You will find the source in `~/lerobot` and an editable install in the active Python environment. All our helper scripts (`start_training.py`, `deploy_policy.py`, …) rely on LeRobot's public API, so you always benefit from upstream upgrades.

• **Training** – `python scripts/start_training.py --dataset <path|hub-id> --config config/lerobot_config.yaml`

• **Evaluating / deploying** – `python scripts/deploy_policy.py --policy-path <checkpoint_folder | hf-hub-id>`

Refer to the official docs for advanced options such as Diffusion Policy, ACT, or SmolVLA architectures.