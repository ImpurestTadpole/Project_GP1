# VR Setup and Usage Guide for SO-101 System

This guide provides detailed instructions for setting up Meta Quest 3 with Unity and using it for robot teleoperation in both simulation and real environments.

## Table of Contents
- [Quest 3 Setup](#quest-3-setup)
- [Unity Setup](#unity-setup)
- [Network Configuration](#network-configuration)
- [Running in Simulation](#running-in-simulation)
- [VR Teleoperation](#vr-teleoperation)
- [Troubleshooting](#troubleshooting)

## Quest 3 Setup

### 1. Initial Quest Setup
1. **Create Meta Account** if you haven't already
   - Visit [meta.com/quest](https://meta.com/quest)
   - Follow account creation process
   
2. **Set Up Developer Mode**
   ```bash
   # Install Meta Quest Developer Hub (MQDH) on your computer
   # Ubuntu: Download .AppImage from Meta's website
   chmod +x MetaQuestDeveloperHub*.AppImage
   ./MetaQuestDeveloperHub*.AppImage
   ```
   
3. **Enable Developer Mode on Quest**
   - Open Meta Quest mobile app
   - Go to Devices → Select your Quest
   - Find Developer Mode and enable it
   - Accept the developer terms

4. **Connect Quest to Computer**
   - Use USB-C cable (USB 3.0 recommended)
   - In Quest, select "Allow USB Debugging"
   - Verify connection:
   ```bash
   # Install ADB tools
   sudo apt install android-tools-adb
   
   # Check connection
   adb devices
   ```

### 2. Quest Development Setup
1. **Install Development Tools**
   ```bash
   # Install required packages
   sudo apt install -y \
       android-sdk \
       openjdk-11-jdk \
       android-sdk-platform-tools
   
   # Set environment variables
   echo 'export ANDROID_HOME=/usr/lib/android-sdk' >> ~/.bashrc
   echo 'export PATH=$PATH:$ANDROID_HOME/platform-tools' >> ~/.bashrc
   source ~/.bashrc
   ```

2. **Configure USB Permissions**
   ```bash
   # Create udev rules for Quest
   sudo tee /etc/udev/rules.d/50-oculus.rules << 'EOF'
   SUBSYSTEM=="usb", ATTR{idVendor}=="2833", MODE="0666", GROUP="plugdev"
   SUBSYSTEM=="usb", ATTR{idVendor}=="2886", MODE="0666", GROUP="plugdev"
   EOF
   
   # Reload rules
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

## Unity Setup

### 1. Install Unity Hub and Editor
```bash
# Add Unity Hub repository
sudo sh -c 'echo "deb https://hub.unity3d.com/linux/repos/deb stable main" > /etc/apt/sources.list.d/unityhub.list'
wget -qO - https://hub.unity3d.com/linux/keys/public | sudo apt-key add -
sudo apt update
sudo apt install unityhub

# Launch Unity Hub and install Unity Editor 2021.3 LTS
unityhub
```

### 2. Configure Unity Project
1. **Open Project**
   ```bash
   # Navigate to unity directory
   cd unity/
   
   # Open with Unity Hub
   unityhub --projectPath $(pwd)
   ```

2. **Install Required Packages**
   - Window → Package Manager
   - Install packages:
     - XR Interaction Toolkit (2.5.2+)
     - XR Plugin Management (4.4.0+)
     - Oculus XR Plugin (4.1.2+)
     - ROS TCP Connector (0.7.0+)

3. **Configure XR Settings**
   - Edit → Project Settings → XR Plug-in Management
   - Enable "Initialize XR on Startup"
   - Under Android tab, enable "Oculus"

4. **Configure Build Settings**
   - File → Build Settings
   - Switch Platform to Android
   - Set parameters:
     ```
     Texture Compression: ASTC
     Target API Level: Android 10.0 (API 29)
     Minimum API Level: Android 10.0 (API 29)
     Scripting Backend: IL2CPP
     Target Architectures: ARM64
     ```

### 3. Configure VR Scene
1. **Open VR Scene**
   - Open `Assets/Scenes/VRTeleop.unity`
   - Verify ROS connection settings:
     ```
     ROS Connection GameObject:
     - ROS IP: Your Ubuntu machine's IP
     - ROS Port: 10000
     ```

2. **Configure Controller Mappings**
   - Edit → Project Settings → Input System
   - Verify VR controller mappings:
     ```
     Left Controller:
     - Trigger: Start/Stop Recording
     - Grip: Reset Position
     - Joystick: Translation
     
     Right Controller:
     - Trigger: Grasp
     - Grip: Mode Switch
     - Joystick: Rotation
     ```

## Network Configuration

### 1. Configure ROS Network
1. **Find Your IP**
   ```bash
   # Get your Ubuntu machine's IP
   ip addr show | grep "inet " | grep -v "127.0.0.1"
   ```

2. **Update ROS Configuration**
   ```bash
   # Edit ROS network settings
   nano config/ros_config.yaml
   
   # Set ROS_DOMAIN_ID and IP
   export ROS_DOMAIN_ID=42
   export ROS_IP=your_ip_address
   ```

### 2. Configure Unity Network
1. **Update Unity Settings**
   - Edit `config/unity_config.json`:
   ```json
   {
     "ros_ip": "your_ubuntu_ip",
     "ros_port": 10000,
     "update_rate": 90
   }
   ```

## Running in Simulation

### 1. Start Simulation Environment
```bash
# Terminal 1: Launch Gazebo simulation
python3 scripts/start_simulation.py

# Wait for Gazebo and RViz to fully load
```

### 2. Start VR Bridge
```bash
# Terminal 2: Launch VR teleoperation
python3 scripts/start_vr_teleop.py

# Wait for "VR Bridge Ready" message
```

### 3. Start Visualization
```bash
# Terminal 3: Launch data visualization
python3 scripts/visualize_data.py
```

## VR Teleoperation

### 1. Deploy to Quest
1. **Build and Deploy**
   ```bash
   # In Unity:
   # File → Build Settings → Build
   # Name: SO101_VR.apk
   
   # Deploy to Quest
   adb install ./Builds/SO101_VR.apk
   ```

2. **Launch on Quest**
   - In Quest: Apps → Unknown Sources
   - Select "SO101 VR Teleoperation"

### 2. Controller Layout
```
Left Controller:
- Trigger (Hold): Start Recording
- Trigger (Release): Stop Recording
- Grip: Reset Robot Position
- Joystick Up/Down: Vertical Translation
- Joystick Left/Right: Horizontal Translation

Right Controller:
- Trigger: Open/Close Gripper
- Grip: Switch Control Mode
- Joystick Up/Down: Pitch Rotation
- Joystick Left/Right: Yaw Rotation
```

### 3. Recording Demonstrations
1. **Start Recording**
   - Position robot using controllers
   - Hold Left Trigger to start recording
   - Perform demonstration
   - Release Trigger to stop recording

2. **Data Location**
   ```bash
   # Recordings are saved to:
   ~/data/raw_episodes/
   
   # Convert recordings:
   python3 scripts/convert_to_lerobot.py
   ```

## Troubleshooting

### Common VR Issues

1. **Quest Not Detected**
   ```bash
   # Check USB connection
   adb devices
   
   # If no devices listed:
   adb kill-server
   adb start-server
   ```

2. **Unity Build Fails**
   - Verify Android SDK/JDK paths in Unity preferences
   - Check build settings match Quest requirements
   - Clear Library folder and rebuild

3. **VR Bridge Connection Issues**
   ```bash
   # Check ROS network
   ros2 topic list  # Should see VR topics
   
   # Verify IP settings match in:
   - config/ros_config.yaml
   - config/unity_config.json
   ```

4. **Controller Tracking Issues**
   - Ensure good lighting conditions
   - Clear Quest guardian boundary
   - Reset controller tracking in Quest settings

### Getting Help
- Check `rostopic echo /vr_status` for bridge status
- Enable debug logging in Unity
- Check `~/.ros/log/` for ROS logs
- Join our Discord for community support

## Next Steps
- Try different robot configurations in `config/robots/`
- Explore example policies in `examples/`
- Customize VR controls in Unity project
- Contribute to our GitHub repository 