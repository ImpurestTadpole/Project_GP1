# LeRobot VR Teleoperation Usage Guide

This guide explains how to use the LeRobot VR teleoperation system for data collection and robot control.

## System Overview

The LeRobot VR teleoperation system consists of:
- **ROS 2 Simulation** - Robot simulation environment
- **Unity VR Application** - VR interface for teleoperation
- **Data Recorder** - Records teleoperation data
- **Data Processing** - Converts and uploads data

## Quick Start

### 1. Start the System

```bash
# Terminal 1: Start simulation
./scripts/start_simulation.sh

# Terminal 2: Start data recorder
./scripts/start_recorder.sh

# Terminal 3: Launch Unity VR app on Quest 3
```

### 2. Basic Operation

1. **Put on Quest 3 headset**
2. **Launch Unity VR application**
3. **Use left controller stick** to move robot
4. **Use right controller trigger** for emergency stop
5. **Use UI buttons** to control recording

## Detailed Usage

### VR Controls

#### Movement Controls
- **Left Controller Stick**
  - Forward/Backward: Move robot forward/backward
  - Left/Right: Rotate robot left/right
  - Deadzone: Small movements are ignored for stability

#### Safety Controls
- **Right Controller Trigger**
  - Emergency Stop: Immediately stops all robot movement
  - Haptic Feedback: Strong vibration indicates emergency stop

#### Recording Controls
- **UI Buttons** (in VR interface)
  - Start Recording: Begin data collection
  - Pause Recording: Temporarily stop data collection
  - Stop Recording: End data collection session

### Data Collection

#### What Gets Recorded
The system automatically records:
- **Robot Commands** (`/cmd_vel`)
  - Linear velocity (forward/backward)
  - Angular velocity (rotation)
- **Camera Data** (`/camera/front/image_raw`)
  - RGB camera feed from robot
- **Depth Data** (`/camera/depth/image_raw`)
  - Depth information from robot sensors
- **Joint States** (`/joint_states`)
  - Robot joint positions and velocities
- **Transform Data** (`/tf`, `/tf_static`)
  - Robot pose and coordinate frames

#### Recording Session
1. **Start Recording**: Data collection begins
2. **Teleoperate**: Control robot normally
3. **Pause/Resume**: Control data collection as needed
4. **Stop Recording**: End session and save data

#### Data Organization
Data is organized by session:
```
~/lerobot_data/
├── 20241224_143022/
│   ├── metadata.json
│   └── data/
│       ├── metadata.yaml
│       └── [bag files]
└── 20241224_150145/
    ├── metadata.json
    └── data/
```

### Data Processing

#### Convert to Parquet
```bash
# Convert bag files to parquet format
./scripts/convert_data.sh ~/lerobot_data/20241224_143022/data ~/lerobot_data/20241224_143022/parquet
```

#### Upload to HuggingFace
```bash
# Set your HuggingFace token
export HUGGINGFACE_TOKEN=your_token_here

# Upload dataset
./scripts/upload_data.sh ~/lerobot_data/20241224_143022/parquet username/lerobot-vr-data
```

## Advanced Usage

### Custom Control Settings

#### Unity Configuration
Modify control parameters in Unity:
- **Max Linear Speed**: Maximum forward/backward speed
- **Max Angular Speed**: Maximum rotation speed
- **Deadzone**: Controller stick deadzone
- **Haptic Intensity**: Haptic feedback strength

#### ROS Configuration
Modify ROS settings:
- **Topic Names**: Customize topic names if needed
- **Publish Rate**: Adjust command publishing frequency
- **Network Settings**: Configure IP and port

### Performance Optimization

#### Reduce Latency
1. **Use local network** between Unity and ROS
2. **Reduce publish rate** if experiencing lag
3. **Optimize network settings** for low latency

#### Battery Life
1. **Reduce haptic intensity** for longer battery life
2. **Use shorter haptic durations**
3. **Optimize Unity rendering settings**

### Safety Features

#### Emergency Stop
- **Automatic**: Right controller trigger
- **Manual**: Unity UI button
- **System**: Script termination

#### Safety Checks
- **Controller Connection**: Warns if controllers disconnected
- **Network Connection**: Warns if ROS connection lost
- **Battery Level**: Monitor Quest 3 battery

## Troubleshooting

### Common Issues

#### VR Controls Not Working
1. **Check controller connection**
2. **Verify XR Interaction Toolkit**
3. **Test controller input in Unity**

#### Recording Not Working
1. **Check ROS connection**
2. **Verify topic names**
3. **Check data directory permissions**

#### Poor Performance
1. **Reduce publish rate**
2. **Optimize network settings**
3. **Check system resources**

### Debug Commands

#### Monitor System Status
```bash
# Check ROS topics
ros2 topic list

# Monitor teleop commands
ros2 topic echo /cmd_vel

# Check recording status
ros2 topic echo /rec_ctrl

# Monitor system resources
htop
```

#### Check Data Recording
```bash
# List recorded sessions
ls ~/lerobot_data/

# Check bag file info
ros2 bag info ~/lerobot_data/session_id/data

# Play back recorded data
ros2 bag play ~/lerobot_data/session_id/data
```

## Best Practices

### Data Collection
1. **Consistent Sessions**: Use similar session lengths
2. **Quality Control**: Review data before processing
3. **Metadata**: Add session notes in metadata.json
4. **Backup**: Regularly backup important data

### Teleoperation
1. **Smooth Movements**: Avoid jerky controller inputs
2. **Safety First**: Use emergency stop when needed
3. **Calibration**: Calibrate controllers regularly
4. **Practice**: Familiarize yourself with controls

### System Maintenance
1. **Regular Updates**: Keep system components updated
2. **Clean Data**: Remove old/unused data
3. **Monitor Resources**: Check system performance
4. **Backup Configuration**: Save important settings

## Integration with Machine Learning

### Data Format
The system provides data in formats suitable for ML:
- **Parquet Files**: Efficient columnar storage
- **HuggingFace Datasets**: Easy ML integration
- **Standardized Topics**: Consistent data structure

### ML Workflow
1. **Collect Data**: Use VR teleoperation
2. **Process Data**: Convert to parquet format
3. **Upload Data**: Share via HuggingFace
4. **Train Models**: Use standard ML frameworks
5. **Deploy Models**: Integrate back to robot

### Example ML Pipeline
```python
from datasets import load_dataset

# Load dataset from HuggingFace
dataset = load_dataset("username/lerobot-vr-data")

# Process data
def process_data(example):
    # Custom processing logic
    return example

# Apply processing
dataset = dataset.map(process_data)

# Train model
# ... ML training code ...
```

## Support and Resources

### Documentation
- [Setup Guide](setup_guide.md)
- [Unity Integration](../unity/README.md)
- [ROS 2 Documentation](https://docs.ros.org/)
- [Unity XR Documentation](https://docs.unity3d.com/Manual/XR.html)

### Community
- [LeRobot GitHub](https://github.com/huggingface/lerobot)
- [ROS 2 Community](https://discourse.ros.org/)
- [Unity XR Community](https://forum.unity.com/forums/xr.31/)

### Getting Help
1. Check troubleshooting section
2. Review error messages carefully
3. Test components individually
4. Check system requirements
5. Contact community forums 