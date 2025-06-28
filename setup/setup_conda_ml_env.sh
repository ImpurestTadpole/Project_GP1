#!/bin/bash
# SO-101 Conda ML Environment Setup
# Creates an isolated Conda environment for machine learning training
# while keeping ROS 2 in system Python for stability

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

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

# Check if conda is installed
if ! command -v conda &> /dev/null; then
    error "Conda not found. Please install Miniconda or Anaconda first."
fi

log "Setting up Conda ML environment for SO-101 system..."

# Environment name
ENV_NAME="lerobot"

# Check if environment already exists
if conda env list | grep -q "^${ENV_NAME} "; then
    warn "Environment '$ENV_NAME' already exists."
    read -p "Do you want to remove and recreate it? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        log "Removing existing environment..."
        conda env remove -n $ENV_NAME -y
    else
        log "Keeping existing environment. Exiting."
        exit 0
    fi
fi

# Create new environment with Python 3.10 (good compatibility with ML libraries)
log "Creating Conda environment '$ENV_NAME' with Python 3.10..."
conda create -n $ENV_NAME python=3.10 -y

# Activate environment
log "Activating environment..."
source $(conda info --base)/etc/profile.d/conda.sh
conda activate $ENV_NAME

# Install PyTorch with CUDA support (if available)
log "Installing PyTorch with CUDA support..."
if command -v nvidia-smi &> /dev/null; then
    log "NVIDIA GPU detected. Installing PyTorch with CUDA 11.8..."
    conda install pytorch torchvision torchaudio pytorch-cuda=11.8 -c pytorch -c nvidia -y
else
    log "No NVIDIA GPU detected. Installing CPU-only PyTorch..."
    conda install pytorch torchvision torchaudio cpuonly -c pytorch -y
fi

# Install core ML packages via conda (faster and more reliable)
log "Installing core ML packages via conda..."
conda install -c conda-forge \
    numpy \
    pandas \
    scipy \
    scikit-learn \
    matplotlib \
    seaborn \
    jupyter \
    notebook \
    ipykernel \
    -y

# Install packages that work better via pip
log "Installing additional packages via pip..."
pip install \
    gymnasium \
    stable-baselines3 \
    transformers \
    datasets \
    accelerate \
    wandb \
    tensorboard \
    opencv-python \
    imageio \
    pillow \
    pyarrow \
    rerun-sdk \
    huggingface_hub \
    pyyaml

# Install HIL-SERL specific dependencies
log "Installing HIL-SERL dependencies..."
log "Attempting to install agentlace from source..."
pip install git+https://github.com/youliangtan/agentlace.git || warn "agentlace installation failed - this is optional for basic functionality"

log "Attempting to install jaxrl from source..."
pip install git+https://github.com/ikostrikov/jaxrl.git || warn "jaxrl installation failed - this is optional, stable-baselines3 can be used instead"

# Install additional RL libraries that are available on PyPI
log "Installing additional RL libraries..."
pip install \
    dm-env \
    dm-tree \
    distrax || warn "Some optional RL libraries failed to install"

# Clone and install LeRobot in the conda environment
log "Installing LeRobot in conda environment..."
LEROBOT_DIR="$HOME/lerobot_conda"

if [ -d "$LEROBOT_DIR" ]; then
    warn "LeRobot directory already exists at $LEROBOT_DIR"
    read -p "Do you want to update it? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        cd "$LEROBOT_DIR"
        git pull
    fi
else
    log "Cloning LeRobot for conda environment..."
    git clone https://github.com/huggingface/lerobot.git "$LEROBOT_DIR"
fi

# Install LeRobot in editable mode
cd "$LEROBOT_DIR"
pip install -e .

# Create activation script
log "Creating environment activation script..."
ACTIVATE_SCRIPT="$HOME/activate_lerobot_env.sh"

cat > "$ACTIVATE_SCRIPT" << 'EOF'
#!/bin/bash
# SO-101 LeRobot Environment Activation Script

# Colors
GREEN='\033[0;32m'
NC='\033[0m'

echo -e "${GREEN}Activating LeRobot ML environment...${NC}"

# Activate conda environment
source $(conda info --base)/etc/profile.d/conda.sh
conda activate lerobot

# Set environment variables for ML training
export LEROBOT_HOME=$HOME/lerobot_conda
export CUDA_VISIBLE_DEVICES=0  # Use first GPU if available
export WANDB_PROJECT="so101_lerobot"

# Verify installation
echo "Environment activated!"
echo "Python: $(which python)"
echo "PyTorch version: $(python -c 'import torch; print(torch.__version__)')"
echo "CUDA available: $(python -c 'import torch; print(torch.cuda.is_available())')"

if command -v nvidia-smi &> /dev/null; then
    echo "GPU info:"
    nvidia-smi --query-gpu=name,memory.total --format=csv,noheader
fi
EOF

chmod +x "$ACTIVATE_SCRIPT"

# Create Jupyter kernel for the environment
log "Creating Jupyter kernel for LeRobot environment..."
python -m ipykernel install --user --name=lerobot --display-name="Python (LeRobot)"

# Create environment info file
log "Creating environment info file..."
INFO_FILE="$HOME/lerobot_env_info.txt"

cat > "$INFO_FILE" << EOF
SO-101 LeRobot Conda Environment Information
==========================================

Environment Name: lerobot
Python Version: $(python --version)
Conda Environment Path: $(conda info --envs | grep lerobot | awk '{print $2}')
LeRobot Installation: $LEROBOT_DIR

Activation Commands:
1. conda activate lerobot
2. OR source $HOME/activate_lerobot_env.sh

Key Packages Installed:
- PyTorch: $(python -c 'import torch; print(torch.__version__)' 2>/dev/null || echo "Not installed")
- LeRobot: $(python -c 'import lerobot; print("Installed")' 2>/dev/null || echo "Not installed")
- Stable-Baselines3: $(python -c 'import stable_baselines3; print(stable_baselines3.__version__)' 2>/dev/null || echo "Not installed")
- Transformers: $(python -c 'import transformers; print(transformers.__version__)' 2>/dev/null || echo "Not installed")

Usage:
- Training: python scripts/start_training.py
- RL Training: python scripts/start_rl_training.py
- Jupyter: jupyter notebook (kernel: Python (LeRobot))

Created: $(date)
EOF

# Test the installation
log "Testing installation..."
python -c "
import torch
import lerobot
import stable_baselines3
import gymnasium
print('✓ All core packages imported successfully!')
print(f'PyTorch version: {torch.__version__}')
print(f'CUDA available: {torch.cuda.is_available()}')
if torch.cuda.is_available():
    print(f'CUDA version: {torch.version.cuda}')
    print(f'GPU count: {torch.cuda.device_count()}')
"

# Deactivate environment
conda deactivate

log "Conda ML environment setup completed successfully!"
echo
echo "=========================================="
echo "SO-101 Conda ML Environment Ready!"
echo "=========================================="
echo
echo "Environment name: $ENV_NAME"
echo "Activation script: $ACTIVATE_SCRIPT"
echo "Environment info: $INFO_FILE"
echo
echo "To activate the environment:"
echo "  conda activate lerobot"
echo "  OR"
echo "  source $HOME/activate_lerobot_env.sh"
echo
echo "To use for training:"
echo "  conda activate lerobot"
echo "  python scripts/start_training.py --config config/lerobot_config.yaml"
echo
echo "To use for RL training:"
echo "  conda activate lerobot"
echo "  python scripts/start_rl_training.py --mode sim"
echo
echo "Jupyter notebook kernel 'Python (LeRobot)' has been created."
echo
log "Setup script completed!" 