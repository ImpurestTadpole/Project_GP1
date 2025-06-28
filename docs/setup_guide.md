# LeRobot VR Teleoperation Setup Guide

This guide provides detailed instructions for setting up the LeRobot VR teleoperation system on Ubuntu.

## Prerequisites

- Ubuntu 22.04 LTS
- Minimum 16GB RAM
- NVIDIA GPU with CUDA support (recommended)
- Meta Quest 3 with latest firmware
- Unity 2022.3 LTS or newer
- Stable network connection

## Installation Order

The setup should be performed in the following order:

1. **System Dependencies** (`setup/install_system.sh`)
2. **ROS 2 Installation** (`setup/install_ros.sh`)
3. **Workspace Setup** (`setup/setup_workspace.sh`)
4. **Python Dependencies** (`setup/install_dependencies.sh`)

## Step-by-Step Installation

### Step 1: System Dependencies

```bash
# Make scripts executable
chmod +x setup/*.sh

# Install system dependencies
./setup/install_system.sh
```

This script installs:
- Basic system packages
- Graphics and GUI dependencies
- Development tools
- Network utilities
- Multimedia codecs
- Python development tools

### Step 2: ROS 2 Installation

```bash
# Install ROS 2 Humble
./setup/install_ros.sh
```

This script installs:
- ROS 2 Humble Desktop
- ROS 2 development tools
- VR and vision packages
- Gazebo simulation
- MoveIt 2

### Step 3: Workspace Setup

```bash
# Set up ROS 2 workspace
./setup/setup_workspace.sh
```

This script:
- Creates ROS 2 workspace
- Clones LeRobot repository
- Creates data recorder package
- Creates data processing package
- Builds the workspace

### Step 4: Python Dependencies

```bash
# Install Python packages
./setup/install_dependencies.sh
```

This script installs:
- Core Python packages (numpy, opencv, etc.)
- ROS 2 Python packages
- Machine learning packages
- Data processing packages
- HuggingFace packages

## Verification

After installation, verify the setup:

```bash
# Check ROS 2 installation
ros2 --version

# Check workspace
source ~/.bashrc
cd ~/ros2_ws
colcon build

# Test LeRobot
ros2 launch lerobot sim_launch.py
```

## Unity Setup

1. **Install Unity 2022.3 LTS**
2. **Create new 3D (URP) project**
3. **Install required packages:**
   - XR Interaction Toolkit
   - ROS TCP Connector
   - Universal Render Pipeline
4. **Import Unity scripts** from `unity/` directory
5. **Configure ROS connection** settings
6. **Build and deploy** to Quest 3

See `unity/README.md` for detailed Unity instructions.

## Usage

### Starting the System

1. **Start Simulation:**
   ```bash
   ./scripts/start_simulation.sh
   ```

2. **Start Data Recorder** (in another terminal):
   ```bash
   ./scripts/start_recorder.sh
   ```

3. **Launch Unity VR application** on Quest 3

### Data Collection

The system records:
- Robot control commands (`/cmd_vel`)
- Camera feed (`/camera/front/image_raw`)
- Depth data (`/camera/depth/image_raw`)
- Joint states (`/joint_states`)
- Transform data (`/tf`, `/tf_static`)

### Data Processing

1. **Convert bag to parquet:**
   ```bash
   ./scripts/convert_data.sh ~/lerobot_data/session_id/data ~/lerobot_data/session_id/parquet
   ```

2. **Upload to HuggingFace:**
   ```bash
   export HUGGINGFACE_TOKEN=your_token_here
   ./scripts/upload_data.sh ~/lerobot_data/session_id/parquet username/dataset_name
   ```

## Troubleshooting

### Common Issues

1. **ROS 2 not found**
   - Run `source /opt/ros/humble/setup.bash`
   - Check if ROS 2 is installed correctly

2. **LeRobot package not found**
   - Run `./setup/setup_workspace.sh`
   - Check if workspace is built correctly

3. **Unity connection failed**
   - Check ROS IP address and port
   - Verify network connectivity
   - Check firewall settings

4. **Controllers not detected**
   - Ensure XR Interaction Toolkit is installed
   - Check Quest 3 developer mode
   - Verify controller firmware

### Debug Commands

```bash
# Check ROS topics
ros2 topic list

# Monitor teleop commands
ros2 topic echo /cmd_vel

# Check recording status
ros2 topic echo /rec_ctrl

# List available packages
ros2 pkg list

# Check system resources
htop
```

## Configuration Files

### ROS Configuration

Edit `config/ros_config.yaml` for ROS settings:

```yaml
ros:
  ip: "127.0.0.1"
  port: 10000
  topics:
    cmd_vel: "/cmd_vel"
    rec_ctrl: "/rec_ctrl"
```

### Unity Configuration

Edit `config/unity_config.json` for Unity settings:

```json
{
  "ros": {
    "ip": "127.0.0.1",
    "port": 10000
  },
  "controls": {
    "maxLinearSpeed": 1.0,
    "maxAngularSpeed": 1.0,
    "deadzone": 0.1,
    "hapticIntensity": 0.5
  }
}
```

## Performance Optimization

1. **Reduce publish rate** if experiencing lag
2. **Use local network** for better latency
3. **Optimize haptic feedback** for battery life
4. **Monitor system resources** during operation

## Security Considerations

1. **Network security** - Use local network only
2. **ROS security** - Configure ROS 2 security if needed
3. **Data privacy** - Secure recorded data appropriately
4. **Access control** - Limit system access

## Support

For additional support:
1. Check the troubleshooting section
2. Review Unity documentation
3. Check ROS 2 documentation
4. Verify system requirements

## Next Steps

After successful setup:
1. Test basic teleoperation
2. Calibrate controllers
3. Set up data collection workflow
4. Configure data processing pipeline
5. Set up HuggingFace integration 