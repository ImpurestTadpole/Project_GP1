#!/bin/bash

# SO-101 VR Teleoperation System Installation Script
# Optimized for Ubuntu 22.04 LTS

set -e  # Exit on error

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'  # No Color

# Logging functions
log() {
    echo -e "${GREEN}[$(date +'%Y-%m-%d %H:%M:%S')] $1${NC}"
}

warn() {
    echo -e "${YELLOW}[WARNING] $1${NC}"
}

error() {
    echo -e "${RED}[ERROR] $1${NC}"
    exit 1
}

# Check if running on Ubuntu 22.04
check_ubuntu_version() {
    log "Checking Ubuntu version..."
    if ! grep -q "Ubuntu 22.04" /etc/os-release; then
        error "This script requires Ubuntu 22.04 LTS. Please use the correct version."
    fi
    log "✓ Ubuntu 22.04 LTS detected"
}

# Install system dependencies
install_system_dependencies() {
    log "Installing system dependencies..."
    
    # Update package lists
    sudo apt update
    
    # Install essential packages
    sudo apt install -y \
        curl \
        wget \
        git \
        build-essential \
        python3 \
        python3-pip \
        python3-venv \
        software-properties-common \
        cmake \
        pkg-config \
        libeigen3-dev \
        libboost-all-dev \
        libgoogle-glog-dev \
        libopencv-dev \
        libusb-1.0-0-dev \
        udev \
        || error "Failed to install system dependencies"
        
    log "✓ System dependencies installed"
}

# Install NVIDIA drivers and CUDA
install_nvidia_drivers() {
    log "Setting up NVIDIA drivers and CUDA..."
    
    # Check if NVIDIA GPU is present
    if ! lspci | grep -i nvidia > /dev/null; then
        warn "No NVIDIA GPU detected. Skipping NVIDIA driver installation."
        return
    }
    
    # Add NVIDIA repository
    sudo add-apt-repository ppa:graphics-drivers/ppa -y
    sudo apt update
    
    # Install NVIDIA drivers
    sudo apt install -y nvidia-driver-535 nvidia-cuda-toolkit
    
    # Verify installation
    if ! nvidia-smi > /dev/null 2>&1; then
        warn "NVIDIA driver installation may have failed. Please reboot and check manually."
    else
        log "✓ NVIDIA drivers installed successfully"
    fi
}

# Install ROS 2 Humble
install_ros2() {
    log "Installing ROS 2 Humble..."
    
    # Add ROS 2 repository
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
        sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    # Update and install
    sudo apt update
    sudo apt install -y \
        ros-humble-desktop \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-ros2-control \
        ros-humble-moveit \
        ros-humble-moveit-ros-planning-interface \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-publisher-gui \
        ros-humble-robot-state-publisher \
        ros-humble-xacro \
        ros-humble-cv-bridge \
        ros-humble-vision-opencv \
        python3-rosdep \
        python3-colcon-common-extensions \
        || error "Failed to install ROS 2"
    
    # Initialize rosdep
    sudo rosdep init || true
    rosdep update
    
    log "✓ ROS 2 Humble installed"
}

# Install Python dependencies
install_python_dependencies() {
    log "Installing Python dependencies..."
    
    # Create virtual environment
    python3 -m venv ~/venv/so101
    source ~/venv/so101/bin/activate
    
    # Upgrade pip
    pip install --upgrade pip
    
    # Install required packages
    pip install \
        numpy \
        scipy \
        matplotlib \
        opencv-python \
        torch \
        torchvision \
        stable-baselines3 \
        gymnasium \
        pyyaml \
        rospkg \
        transforms3d \
        pyrealsense2 \
        || error "Failed to install Python packages"
    
    log "✓ Python dependencies installed"
}

# Install LeRobot
install_lerobot() {
    log "Installing LeRobot..."
    
    # Clone LeRobot repository
    if [ ! -d "lerobot" ]; then
        git clone https://github.com/huggingface/lerobot.git
        cd lerobot
        pip install -e .
        cd ..
    fi
    
    log "✓ LeRobot installed"
}

# Set up ROS workspace
setup_ros_workspace() {
    log "Setting up ROS workspace..."
    
    # Create workspace directory
    mkdir -p ~/ros_ws/src
    cd ~/ros_ws
    
    # Copy ROS packages
    cp -r $PROJECT_DIR/ros_ws/src/* src/
    
    # Install dependencies
    rosdep install --from-paths src --ignore-src -r -y
    
    # Build workspace
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    
    log "✓ ROS workspace set up"
}

# Configure environment
configure_environment() {
    log "Configuring environment..."
    
    # Add environment variables to .bashrc
    cat >> ~/.bashrc << 'EOF'

# SO-101 VR Teleoperation System Environment
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
source ~/venv/so101/bin/activate

export SO101_PROJECT_DIR=~/so101_vr_system
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros_ws/src/so101_sim/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/ros_ws/src/so101_sim/worlds
export PYTHONPATH=$PYTHONPATH:~/ros_ws/src
export ROS_DOMAIN_ID=42
EOF

    # Create data directories
    mkdir -p ~/data/{sim_episodes,real_episodes,lerobot_episodes}
    mkdir -p ~/models/lerobot
    mkdir -p ~/logs/lerobot
    mkdir -p ~/checkpoints/lerobot
    
    log "✓ Environment configured"
}

# Main installation function
main() {
    log "Starting SO-101 VR Teleoperation System installation..."
    
    # Store project directory
    export PROJECT_DIR=$(pwd)
    
    # Run installation steps
    check_ubuntu_version
    install_system_dependencies
    install_nvidia_drivers
    install_ros2
    install_python_dependencies
    install_lerobot
    setup_ros_workspace
    configure_environment
    
    log "Installation completed successfully!"
    log "Please restart your terminal or run: source ~/.bashrc"
    log "Then run: python3 scripts/verify_installation.py"
}

# Run main installation
main "$@"
