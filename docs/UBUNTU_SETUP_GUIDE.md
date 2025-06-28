# SO-101 VR Teleoperation System - Ubuntu Setup Guide

## Overview

This guide provides instructions for creating a bootable Ubuntu system with the SO-101 VR teleoperation system pre-installed. The system is designed to run on Ubuntu 22.04 LTS and provides a complete environment for VR teleoperation and reinforcement learning.

## Installation Options

### Option 1: Clean Installation (Recommended)

For a fresh Ubuntu 22.04 installation:

1. **Download Ubuntu 22.04 LTS**
   ```bash
   wget https://releases.ubuntu.com/22.04/ubuntu-22.04.3-desktop-amd64.iso
   ```

2. **Create bootable USB**
   ```bash
   sudo dd if=ubuntu-22.04.3-desktop-amd64.iso of=/dev/sdX bs=4M status=progress
   ```
   Replace `/dev/sdX` with your USB device.

3. **Boot from USB and install Ubuntu**

4. **Run the installation script**
   ```bash
   wget https://raw.githubusercontent.com/your-repo/so101-vr-system/main/install_all.sh
   chmod +x install_all.sh
   ./install_all.sh
   ```

### Option 2: Minimal Installation

For existing Ubuntu systems:

1. **Run the clean installation script**
   ```bash
   wget https://raw.githubusercontent.com/your-repo/so101-vr-system/main/clean_install.sh
   chmod +x clean_install.sh
   ./clean_install.sh
   ```

2. **Install minimal system**
   ```bash
   cd ~/so101_vr_system
   ./install_minimal.sh
   ```

### Option 3: Custom Ubuntu Image

Create a bootable ISO with the system pre-installed:

1. **Run the image creation script**
   ```bash
   sudo wget https://raw.githubusercontent.com/your-repo/so101-vr-system/main/create_ubuntu_image.sh
   sudo chmod +x create_ubuntu_image.sh
   sudo ./create_ubuntu_image.sh
   ```

2. **Create bootable USB from custom ISO**
   ```bash
   sudo dd if=so101-vr-ubuntu-22.04.3.iso of=/dev/sdX bs=4M status=progress
   ```

## System Requirements

### Hardware Requirements

- **CPU**: Intel i5/AMD Ryzen 5 or better (4+ cores)
- **RAM**: 16GB minimum, 32GB recommended
- **Storage**: 50GB free space minimum
- **GPU**: NVIDIA GTX 1060 or better (for VR and training)
- **VR Headset**: Meta Quest 3 (recommended) or compatible
- **USB**: USB 3.0 ports for VR headset

### Software Requirements

- **OS**: Ubuntu 22.04 LTS
- **ROS**: ROS 2 Humble
- **Python**: 3.8+
- **VR Runtime**: SteamVR or Oculus software

## Installation Process

### Step 1: System Preparation

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install essential packages
sudo apt install -y \
    curl \
    wget \
    git \
    build-essential \
    python3 \
    python3-pip \
    software-properties-common
```

### Step 2: ROS 2 Installation

```bash
# Add ROS 2 repository
sudo apt update && sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

# Install ROS 2 Humble
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-dev-tools
```

### Step 3: Simulation Environment

```bash
# Install Gazebo and MoveIt
sudo apt install -y \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-moveit \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-rviz2
```

### Step 4: Python Dependencies

```bash
# Install Python packages
pip3 install --user \
    numpy \
    opencv-python \
    matplotlib \
    scipy \
    scikit-learn \
    torch \
    torchvision \
    lerobot \
    stable-baselines3 \
    gymnasium \
    pyyaml \
    rospkg
```

### Step 5: VR Support

```bash
# Install Steam (for VR runtime)
sudo apt install -y steam

# Install Oculus support
sudo apt install -y oculus-udev oculus-client

# Install Unity Hub
wget -qO- https://hub.unity3d.com/linux/repos/deb/unity-hub.gpg.key | sudo apt-key add -
sudo sh -c 'echo "deb https://hub.unity3d.com/linux/repos/deb stable main" > /etc/apt/sources.list.d/unity-hub.list'
sudo apt update
sudo apt install -y unity-hub
```

### Step 6: Project Setup

```bash
# Create project directory
mkdir -p ~/so101_vr_system
cd ~/so101_vr_system

# Clone or copy project files
git clone https://github.com/your-repo/so101-vr-system.git .

# Create ROS workspace
mkdir -p ~/ros_ws/src
cd ~/ros_ws

# Copy ROS packages
cp -r ~/so101_vr_system/ros_ws/src/* src/

# Build workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### Step 7: Environment Setup

```bash
# Add environment variables to .bashrc
cat >> ~/.bashrc << 'EOF'

# SO-101 VR System Environment
export SO101_PROJECT_DIR=~/so101_vr_system
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros_ws/src/so101_sim/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/ros_ws/src/so101_sim/worlds
export PYTHONPATH=$PYTHONPATH:~/ros_ws/src
export ROS_DOMAIN_ID=42
EOF

# Source environment
source ~/.bashrc
```

### Step 8: Data Directories

```bash
# Create data directories
mkdir -p ~/data/{sim_episodes,real_episodes,lerobot_episodes}
mkdir -p ~/models/lerobot
mkdir -p ~/logs/lerobot
mkdir -p ~/checkpoints/lerobot
```

## Verification

### System Check

```bash
# Run verification script
cd ~/so101_vr_system
./verify_installation.sh
```

### Manual Verification

```bash
# Check ROS 2
ros2 --version

# Check Gazebo
gazebo --version

# Check Python packages
python3 -c "import torch; print(f'PyTorch {torch.__version__}')"
python3 -c "import lerobot; print('LeRobot available')"

# Check workspace
ls ~/ros_ws/src/
```

## Usage

### Quick Start

```bash
# Start full pipeline
cd ~/so101_vr_system
python3 scripts/start_full_pipeline.py
```

### Individual Components

```bash
# Simulation only
python3 scripts/start_simulation.py

# VR teleoperation
python3 scripts/start_vr_teleop.py

# Training
python3 scripts/start_training.py --mode train

# Data collection
python3 scripts/start_training.py --mode collect
```

### VR Setup

1. **Connect Quest 3**
   - Enable Developer Mode in Quest 3
   - Connect via USB or wireless
   - Install SteamVR or Oculus software

2. **Unity Configuration**
   - Open Unity Hub
   - Add Unity project from `unity/` directory
   - Install ROS-TCP-Connector package

3. **Calibration**
   - Run calibration script
   - Adjust VR controller mappings
   - Set workspace boundaries

## Troubleshooting

### Common Issues

1. **ROS not found**
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Workspace not found**
   ```bash
   source ~/ros_ws/install/setup.bash
   ```

3. **Python package errors**
   ```bash
   pip3 install --user <package_name>
   ```

4. **VR headset not detected**
   - Check USB connection
   - Enable Developer Mode
   - Install VR runtime software

5. **Gazebo not starting**
   ```bash
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros_ws/src/so101_sim/models
   gazebo
   ```

### Performance Issues

1. **Reduce physics update rate**
   ```bash
   export GAZEBO_UPDATE_RATE=30
   ```

2. **Use headless mode for training**
   ```bash
   python3 scripts/start_training.py --headless
   ```

3. **Limit episode length**
   - Edit config files to reduce episode duration

### Network Issues

1. **ROS communication problems**
   ```bash
   export ROS_DOMAIN_ID=42
   ```

2. **Unity connection issues**
   - Check firewall settings
   - Verify IP address in Unity config

## Maintenance

### Updates

```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Update Python packages
pip3 install --user --upgrade lerobot torch stable-baselines3

# Update ROS workspace
cd ~/ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### Backup

```bash
# Backup data
tar -czf so101_backup_$(date +%Y%m%d).tar.gz ~/data ~/models ~/logs

# Backup configuration
cp -r ~/so101_vr_system/config ~/so101_backup_config
```

### Cleanup

```bash
# Remove old data
rm -rf ~/data/sim_episodes/*
rm -rf ~/data/real_episodes/*
rm -rf ~/logs/lerobot/*

# Clean ROS workspace
cd ~/ros_ws
rm -rf build/ install/ log/
```

## Support

### Documentation

- **Main README**: `~/so101_vr_system/README.md`
- **Quick Start**: `~/so101_vr_system/QUICK_START.md`
- **Configuration**: `~/so101_vr_system/config/`

### Logs

- **System logs**: `journalctl -u so101-vr.service`
- **ROS logs**: `ros2 log`
- **Training logs**: `~/logs/lerobot/`

### Community

- **GitHub Issues**: Report bugs and request features
- **Discord**: Join community for support
- **Documentation**: Check wiki for detailed guides

## License

This system is licensed under the Apache 2.0 License. See LICENSE file for details.

## Acknowledgments

- ROS 2 community for the robotics framework
- Unity Technologies for VR support
- Hugging Face for LeRobot framework
- Meta for Quest 3 VR hardware 