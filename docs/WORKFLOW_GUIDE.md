# SO-101 Hybrid Python Environment Workflow Guide

This guide explains the **recommended hybrid approach** for running the SO-101 system, which separates ROS operations (system Python) from ML training (Conda environment).

## 🏗️ Architecture Overview

```
Ubuntu System:
├── ROS 2 Humble (system-wide)           ← Robot control, hardware drivers
├── System Python 3.10                  ← ROS nodes, real-time operations
└── Robot hardware interfaces

Conda Environment "lerobot":
├── PyTorch + CUDA                       ← GPU-optimized ML training
├── LeRobot                             ← Isolated ML dependencies
├── Training libraries                   ← No conflicts with system
└── Jupyter notebooks
```

## 📋 Installation Steps

### 1. System-Wide Installation (Required)
```bash
# Clone repository
git clone <repository-url>
cd <project-directory>

# Run main installer (installs ROS, system dependencies)
chmod +x install_all.sh
./install_all.sh
source ~/.bashrc
```

### 2. Conda ML Environment (Recommended)
```bash
# Install Miniconda (if not already installed)
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh

# Set up isolated ML environment
bash setup/setup_conda_ml_env.sh
```

## 🔄 Daily Workflow

### Robot Operation & Data Collection (System Python)

**Terminal 1: Simulation/Robot**
```bash
# System Python - no conda activation needed
python3 scripts/start_simulation.py
# OR for real robot:
# ros2 launch so101_bringup real.launch.py
```

**Terminal 2: VR Teleoperation**
```bash
# System Python - handles real-time VR input
python3 scripts/start_vr_teleop.py
```

**Terminal 3: Data Visualization**
```bash
# System Python - real-time data streaming
python3 scripts/visualize_data.py
```

### ML Training & Development (Conda Environment)

**Terminal 4: Training**
```bash
# Activate ML environment
conda activate lerobot

# Imitation Learning
python scripts/start_training.py --config config/lerobot_config.yaml

# OR Reinforcement Learning
python scripts/start_rl_training.py --mode sim
```

**Terminal 5: Development/Jupyter**
```bash
# Activate ML environment
conda activate lerobot

# Start Jupyter with LeRobot kernel
jupyter notebook
# Select "Python (LeRobot)" kernel
```

### Policy Deployment (Either Environment)

**Option 1: System Python** (if LeRobot installed system-wide)
```bash
python3 scripts/deploy_policy.py --policy-path path/to/policy
```

**Option 2: Conda Environment** (recommended for GPU inference)
```bash
conda activate lerobot
python scripts/deploy_policy.py --policy-path path/to/policy
```

## 🎯 When to Use Which Environment

### Use System Python For:
- ✅ **ROS node execution** (real-time, stable)
- ✅ **Robot hardware control** (low-latency)
- ✅ **VR teleoperation** (real-time input processing)
- ✅ **Data collection** (system integration)
- ✅ **Simulation launch** (ROS-integrated)

### Use Conda Environment For:
- 🧠 **ML model training** (GPU-optimized)
- 🧠 **Policy development** (isolated dependencies)
- 🧠 **Data analysis** (Jupyter notebooks)
- 🧠 **Experiment tracking** (wandb, tensorboard)
- 🧠 **Model inference** (GPU acceleration)

## 🔧 Environment Management

### Conda Environment Commands
```bash
# Activate
conda activate lerobot

# Quick activation with environment info
source ~/activate_lerobot_env.sh

# Deactivate
conda deactivate

# List environments
conda env list

# Update packages
conda activate lerobot
pip install --upgrade lerobot transformers

# Remove environment (if needed)
conda env remove -n lerobot
```

### Environment Information
```bash
# Check current environment
which python
python --version

# Verify ML packages (in conda env)
conda activate lerobot
python -c "import torch; print(f'PyTorch: {torch.__version__}, CUDA: {torch.cuda.is_available()}')"

# Check ROS (system Python)
conda deactivate
python3 -c "import rclpy; print('ROS 2 available')"
```

## 🚀 Complete Workflow Example

### Day 1: Data Collection
```bash
# Terminal 1: Start simulation
python3 scripts/start_simulation.py

# Terminal 2: VR teleoperation
python3 scripts/start_vr_teleop.py

# Terminal 3: Visualization
python3 scripts/visualize_data.py

# Collect demonstrations using VR controllers
# Data saved to ~/data/raw_episodes/
```

### Day 2: Training
```bash
# Convert data to LeRobot format (system Python)
python3 scripts/convert_to_lerobot.py

# Train policy (conda environment)
conda activate lerobot
python scripts/start_training.py --config config/lerobot_config.yaml

# OR train with RL
python scripts/start_rl_training.py --mode sim
```

### Day 3: Deployment
```bash
# Terminal 1: Start environment
python3 scripts/start_simulation.py

# Terminal 2: Deploy trained policy
conda activate lerobot
python scripts/deploy_policy.py --policy-path ~/checkpoints/lerobot/best_model
```

## 🎛️ Advanced Tips

### GPU Memory Management
```bash
# Monitor GPU usage during training
watch -n 1 nvidia-smi

# Set GPU memory limit (in conda env)
export CUDA_VISIBLE_DEVICES=0
export PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:512
```

### Experiment Tracking
```bash
# Activate conda environment
conda activate lerobot

# Login to wandb (one time)
wandb login

# Training with experiment tracking
python scripts/start_training.py --config config/lerobot_config.yaml
```

### Development Workflow
```bash
# Edit code in any editor
code ros_ws/src/so101_lerobot/

# Test ROS nodes (system Python)
python3 -m pytest tests/

# Test ML code (conda environment)  
conda activate lerobot
python -m pytest ml_tests/
```

## 🔍 Troubleshooting

### Common Issues

**"ModuleNotFoundError: No module named 'rclpy'"**
- Solution: Use system Python for ROS operations
- Don't activate conda when running ROS nodes

**"CUDA out of memory"**
- Solution: Reduce batch size in config files
- Use `export PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:512`

**"Import Error: lerobot"**
- In system Python: Check if installed with `pip3 show lerobot`
- In conda: Activate environment first `conda activate lerobot`

### Environment Conflicts
```bash
# Check which Python is active
which python
echo $CONDA_DEFAULT_ENV

# Reset to system Python
conda deactivate
export PATH="/usr/bin:$PATH"
```

## 📊 Performance Benefits

### Training Speed Comparison
- **System Python**: Good for development, slower training
- **Conda Environment**: 2-3x faster training with optimized packages

### Memory Usage
- **System Python**: Lower overhead, better for real-time operations
- **Conda Environment**: Higher memory usage, better for batch processing

### Stability
- **System Python**: More stable for ROS operations
- **Conda Environment**: Isolated, won't break system dependencies

## 🎉 Summary

This hybrid approach gives you:
- ✅ **Stable robot operations** (system Python)
- ✅ **Fast ML training** (conda environment)  
- ✅ **No dependency conflicts**
- ✅ **Easy maintenance and updates**
- ✅ **Professional development workflow**

The key is knowing when to use which environment. **Robot operations = system Python, ML training = conda environment!** 