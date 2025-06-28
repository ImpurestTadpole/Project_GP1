# LeKiWi Dual-Arm VR Teleoperation Guide
## Shirt-in-Box Task Data Collection

This guide will walk you through setting up and collecting data for a shirt folding and placement task using the LeKiWi dual-arm robot and Meta Quest 3 VR controllers.

## Prerequisites

- LeKiWi dual-arm robot properly installed and calibrated
- Meta Quest 3 headset with controllers
- ROS 2 Humble installed on Ubuntu 22.04
- Project workspace properly set up
- Safety training completed
- At least 2m x 2m clear space for VR operation

## Hardware Setup

### 1. Robot Setup
1. Power on the LeKiWi robot
2. Ensure the e-stop is easily accessible
3. Clear the workspace around the robot (minimum 1.5m radius)
4. Place the collection box at the designated position (usually right side)
5. Stack clean shirts on the left side of the workspace

### 2. VR Setup
1. Charge Quest 3 controllers and headset
2. Clear VR play area (minimum 2m x 2m)
3. Set up Quest 3 guardian boundaries
4. Launch Quest 3 in developer mode
5. Connect Quest 3 to your development PC via USB-C cable

### 3. Camera Setup
1. Mount RealSense D435i cameras:
   - One overlooking the workspace from above (~1.5m height)
   - Two cameras on robot torso for close-up views
2. Connect all cameras via USB 3.0 to the control PC
3. Verify camera feeds using:
   ```bash
   ros2 run so101_camera camera_test.py
   ```

## Software Setup

### 1. Start Robot Control Stack
```bash
# Terminal 1: Launch robot drivers and hardware interface
ros2 launch so101_bringup real.launch.py robot:=lekiwi

# Terminal 2: Launch cameras and perception
ros2 launch so101_camera camera.launch.py config:=lekiwi_3cam.yaml

# Terminal 3: Launch VR bridge
ros2 launch so101_teleop_vr vr_teleop.launch.py robot:=lekiwi
```

### 2. Start Data Collection
```bash
# Terminal 4: Launch data recorder
ros2 launch so101_lerobot record.launch.py task:=shirt_box
```

### 3. Launch Visualization
```bash
# Terminal 5: Launch RViz and Rerun
ros2 launch so101_bringup viz.launch.py config:=lekiwi_vr.rviz
```

## VR Controller Mapping

### Left Controller
- Joystick: Translate left arm
- Grip: Close/open left gripper
- Trigger: Enable left arm movement
- X Button: Emergency stop
- Y Button: Reset left arm pose

### Right Controller
- Joystick: Translate right arm
- Grip: Close/open right gripper
- Trigger: Enable right arm movement
- A Button: Start/stop recording
- B Button: Reset right arm pose

## Data Collection Procedure

### 1. Initial Setup
1. Put on the Quest 3 headset
2. Verify you can see both virtual robot arms
3. Test controller mappings
4. Ensure data recording is ready

### 2. Collecting a Single Episode
1. Press 'A' to start recording
2. Pick up shirt with both arms (typically corners)
3. Lift and unfold shirt in air
4. Fold shirt using coordinated arm movements:
   - Bring corners together
   - Fold sleeves inward
   - Complete final fold
5. Move folded shirt over box
6. Gently place in box
7. Release grippers and retract arms
8. Press 'A' to stop recording

### 3. Best Practices
- Maintain consistent folding pattern
- Keep movements smooth and deliberate
- Avoid rapid or jerky motions
- Verify shirt placement before releasing
- Record at least 50 successful episodes
- Mix up initial shirt positions and orientations

## Safety Guidelines

1. **Always** maintain visual contact with the robot
2. Keep e-stop within reach
3. Stop if anything feels wrong
4. Maintain safe distance from robot
5. Follow all facility safety protocols

## Troubleshooting

### VR Issues
- **Lost Tracking**: Remove headset, ensure good lighting
- **Controller Drift**: Reset controller pose with Y/B buttons
- **Delayed Response**: Check USB connection and PC load

### Robot Issues
- **Stiff Movement**: Check speed settings in config
- **Gripper Problems**: Verify air pressure
- **Position Errors**: Use reset pose functions

### Recording Issues
- **Missing Data**: Check disk space
- **Dropped Frames**: Reduce camera resolution
- **Failed Save**: Verify permissions

## Data Verification

After collecting episodes:
1. Check recorded data:
   ```bash
   ros2 run so101_lerobot verify_episodes.py --task shirt_box
   ```
2. Visualize random episodes:
   ```bash
   ros2 run so101_lerobot play_episode.py --random
   ```
3. Convert to LeRobot format:
   ```bash
   ros2 run so101_lerobot convert_to_lerobot.py --task shirt_box
   ```

## Next Steps

After data collection:
1. Review episodes for quality
2. Clean up any failed recordings
3. Backup raw data
4. Begin training process:
   ```bash
   python3 scripts/start_training.py --task shirt_box --model dexnet
   ```

## Support

For issues or questions:
- Check troubleshooting guide
- Review system logs
- Contact robotics support team
- File issue on project repository

## References

- LeKiWi Robot Manual
- Quest 3 Developer Guide
- ROS 2 Documentation
- Project API Reference 