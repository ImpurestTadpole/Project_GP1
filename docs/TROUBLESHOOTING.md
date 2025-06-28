# SO-101 VR Teleoperation System - Troubleshooting Guide

This guide covers common issues and their solutions for the SO-101 VR Teleoperation System.

## Installation Issues

### LeRobot Git Repository Error

**Error Message:**
```
fatal: not a git repository (or any of the parent directories): .git
```

**Cause:** The LeRobot directory exists but is not a proper Git repository (possibly from a failed previous installation).

**Solution:**
```bash
# Run the automated fix script
chmod +x fix_lerobot_install.sh
./fix_lerobot_install.sh
```

**Manual Solution:**
```bash
# Remove the corrupted directory
rm -rf ~/lerobot

# Re-clone LeRobot
git clone https://github.com/huggingface/lerobot.git ~/lerobot
cd ~/lerobot
pip3 install --user -e .
```

### Python Package Installation Failures

**Error:** Package installation fails with permission errors.

**Solution:**
```bash
# Use user installation
pip3 install --user package_name

# Or create a virtual environment
python3 -m venv ~/venv/so101
source ~/venv/so101/bin/activate
pip install package_name
```

### ROS 2 Installation Issues

**Error:** ROS 2 packages not found or installation fails.

**Solution:**
```bash
# Update package lists
sudo apt update

# Install ROS 2 key manually
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Re-run the installation
./install_all.sh
```

## Runtime Issues

### VR Headset Not Detected

**Symptoms:** VR interface doesn't respond to headset movements.

**Solutions:**
1. **Check USB Connection:**
   ```bash
   lsusb | grep -i oculus  # or Meta
   ```

2. **Install VR Runtime:**
   ```bash
   # Install SteamVR or Oculus software
   sudo apt install steam
   ```

3. **Check Permissions:**
   ```bash
   sudo usermod -a -G plugdev $USER
   # Log out and back in
   ```

### Camera Not Working

**Symptoms:** No camera feed in visualization or black screen.

**Solutions:**
1. **Check Camera Connection:**
   ```bash
   v4l2-ctl --list-devices
   ```

2. **Test Camera:**
   ```bash
   # Test USB camera
   cheese  # or
   ffplay /dev/video0
   
   # Test RealSense
   realsense-viewer
   ```

3. **Fix Permissions:**
   ```bash
   sudo usermod -a -G video $USER
   ```

### ROS 2 Nodes Not Communicating

**Symptoms:** Nodes start but don't exchange data.

**Solutions:**
1. **Check ROS Domain:**
   ```bash
   echo $ROS_DOMAIN_ID  # Should be 42
   export ROS_DOMAIN_ID=42
   ```

2. **Check Network:**
   ```bash
   ros2 node list
   ros2 topic list
   ros2 topic echo /topic_name
   ```

3. **Restart ROS Daemon:**
   ```bash
   ros2 daemon stop
   ros2 daemon start
   ```

### Policy Training Fails

**Symptoms:** Training crashes or produces poor results.

**Solutions:**
1. **Check Data Format:**
   ```bash
   python3 scripts/convert_to_lerobot.py --verify
   ```

2. **Verify GPU Setup:**
   ```bash
   nvidia-smi
   python3 -c "import torch; print(torch.cuda.is_available())"
   ```

3. **Reduce Batch Size:**
   Edit `config/lerobot_config.yaml` and reduce `batch_size`

### Simulation Won't Start

**Symptoms:** Gazebo crashes or doesn't display robot.

**Solutions:**
1. **Check Gazebo Installation:**
   ```bash
   gazebo --version
   ```

2. **Reset Gazebo:**
   ```bash
   rm -rf ~/.gazebo/models
   gazebo --reset-all
   ```

3. **Check Graphics:**
   ```bash
   # For headless systems
   export DISPLAY=:0
   # or
   gazebo --headless
   ```

## Performance Issues

### Slow Training

**Solutions:**
1. **Use GPU:**
   ```bash
   export CUDA_VISIBLE_DEVICES=0
   ```

2. **Increase Workers:**
   Edit config files to increase `num_workers`

3. **Use SSD Storage:**
   Move data to SSD: `~/data/` → `/path/to/ssd/data/`

### High Latency in VR

**Solutions:**
1. **Reduce Image Resolution:**
   Edit camera settings in launch files

2. **Optimize Network:**
   ```bash
   # Use wired connection for VR streaming
   # Reduce network traffic
   ```

3. **Close Unnecessary Programs:**
   ```bash
   htop  # Check CPU/memory usage
   ```

## Verification Commands

Run these commands to verify your installation:

```bash
# Check ROS 2
ros2 --version

# Check Python packages
python3 -c "import lerobot, torch, cv2; print('All packages OK')"

# Check hardware
nvidia-smi  # GPU
v4l2-ctl --list-devices  # Cameras
lsusb  # USB devices

# Test basic functionality
python3 scripts/start_teleop.py --control keyboard --robot so101
```

## Getting Help

If you encounter issues not covered here:

1. **Check Logs:**
   ```bash
   journalctl -u so101-vr.service  # System service logs
   ros2 log  # ROS logs
   ```

2. **Enable Debug Mode:**
   ```bash
   export ROS_LOG_LEVEL=DEBUG
   export PYTHONPATH=$PYTHONPATH:~/ros_ws/src
   ```

3. **Create Issue Report:**
   Include:
   - Error messages
   - System info (`uname -a`, `lsb_release -a`)
   - Hardware info (`lscpu`, `lsusb`, `nvidia-smi`)
   - Installation log

## Common File Locations

- **Logs:** `~/logs/lerobot/`
- **Data:** `~/data/`
- **Models:** `~/models/lerobot/`
- **Config:** `~/ros_ws/config/`
- **ROS Workspace:** `~/ros_ws/`
- **LeRobot Source:** `~/lerobot/` 