#!/bin/bash
# Deploy Policy Script - Shell wrapper for deploy_policy.py

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
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

# Check if policy path is provided
if [ $# -lt 1 ]; then
    error "Usage: $0 <policy_path> [additional_args...]"
fi

POLICY_PATH="$1"
shift  # Remove first argument, keep the rest

# Check if policy file exists
if [ ! -f "$POLICY_PATH" ]; then
    error "Policy file not found: $POLICY_PATH"
fi

log "Deploying policy: $POLICY_PATH"

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS environment if available
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    log "Sourced ROS Humble environment"
fi

if [ -f "$HOME/ros_ws/install/setup.bash" ]; then
    source "$HOME/ros_ws/install/setup.bash"
    log "Sourced local ROS workspace"
fi

# Run the Python deployment script
log "Starting policy deployment..."
python3 "$SCRIPT_DIR/deploy_policy.py" --policy-path "$POLICY_PATH" "$@"

if [ $? -eq 0 ]; then
    log "Policy deployment completed successfully"
else
    error "Policy deployment failed"
fi 