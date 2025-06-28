# SO-101 VR Teleoperation System - Quick Start Guide

This guide provides quick instructions for getting started with the SO-101 VR Teleoperation System on Ubuntu 22.04 LTS.

## Prerequisites

- Ubuntu 22.04 LTS (fresh installation recommended)
- NVIDIA GPU (recommended for training)
- Meta Quest 3 or compatible VR headset
- USB 3.0 ports
- Internet connection

## Installation

1. **Clone the Repository**
   ```bash
   git clone https://github.com/your-org/so101-vr-system.git
   cd so101-vr-system
   ```

2. **Run Installation Script**
   ```bash
   chmod +x install_all.sh
   ./install_all.sh
   ```

3. **Verify Installation**
   ```bash
   # Open a new terminal or source environment
   source ~/.bashrc
   
   # Run verification script
   python3 scripts/verify_installation.py
   ```

## Running the System

### Option 1: Simulation Mode

1. **Terminal 1: Launch Simulation**
   ```bash
   python3 scripts/start_simulation.py
   ```

2. **Terminal 2: Start VR Teleoperation**
   ```bash
   python3 scripts/start_vr_teleop.py
   ```

3. **Terminal 3: Launch Visualization**
   ```bash
   python3 scripts/visualize_data.py
   ```

### Option 2: Real Robot Mode

1. **Terminal 1: Launch Robot**
   ```bash
   ros2 launch so101_bringup real.launch.py
   ```

2. **Terminal 2: Start VR Teleoperation with Camera**
   ```bash
   # For USB camera (default)
   python3 scripts/start_vr_teleop.py
   
   # For RealSense camera
   python3 scripts/start_vr_teleop.py camera_type:=realsense
   ```

3. **Terminal 3: Launch Visualization**
   ```bash
   python3 scripts/visualize_data.py
   ```

## Data Collection & Training

1. **Record Demonstrations**
   - Use VR controllers to demonstrate tasks
   - Press **R** to start recording
   - Press **S** to stop recording
   - Data is saved to `~/data/raw_episodes/`

2. **Convert Data**
   ```bash
   python3 scripts/convert_to_lerobot.py
   ```

3. **Train Policy**
   ```bash
   # Option 1: Imitation Learning
   python3 scripts/start_training.py
   
   # Option 2: Simple RL
   python3 scripts/start_simple_rl_training.py
   
   # Option 3: Advanced RL (HIL-SERL)
   python3 scripts/start_rl_training.py
   ```

## Deploy Trained Policy

1. **Start Environment**
   ```bash
   # For simulation
   python3 scripts/start_simulation.py
   
   # For real robot
   ros2 launch so101_bringup real.launch.py
   ```

2. **Deploy Policy**
   ```bash
   # For simulation
   python3 scripts/deploy_policy.py --policy-path path/to/your/policy
   
   # For real robot with USB camera
   python3 scripts/deploy_policy.py --policy-path path/to/your/policy --camera-type usb
   
   # For real robot with RealSense
   python3 scripts/deploy_policy.py --policy-path path/to/your/policy --camera-type realsense
   ```

## Troubleshooting

### Common Issues

1. **ROS 2 Not Found**
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros_ws/install/setup.bash
   ```

2. **Python Environment Issues**
   ```bash
   source ~/venv/so101/bin/activate
   ```

3. **NVIDIA/CUDA Problems**
   ```bash
   # Check NVIDIA driver
   nvidia-smi
   
   # Check CUDA
   python3 -c "import torch; print(torch.cuda.is_available())"
   ```

4. **Camera Issues**
   - Check USB connection
   - Verify camera permissions: `ls -l /dev/video*`
   - For RealSense: `realsense-viewer` to test camera

### Getting Help

- Check `docs/TROUBLESHOOTING.md` for detailed solutions
- Visit our GitHub repository for issues and updates
- Join our Discord community for support

## Next Steps

- Read the full documentation in `docs/`
- Explore example policies in `examples/`
- Try different robot configurations in `config/robots/`
- Customize training parameters in `config/rl_config.yaml` 