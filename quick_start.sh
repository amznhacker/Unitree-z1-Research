#!/bin/bash

# Unitree Z1 Quick Start - One Command Launch
# Usage: ./quick_start.sh [keyboard|xbox|demo|draw|chess|wave]

WORKSPACE_DIR="$HOME/catkin_ws"
CONTROL_METHOD="${1:-keyboard}"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

print_status() {
    echo -e "${BLUE}[Z1]${NC} $1"
}

cleanup() {
    print_status "Shutting down..."
    pkill -f ros || true
    pkill -f gazebo || true
    exit 0
}

trap cleanup SIGINT SIGTERM

# Check if workspace exists
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "Error: Workspace not found at $WORKSPACE_DIR"
    echo "Run ./setup_and_run.sh --install-ros first"
    exit 1
fi

# Source workspace
source "$WORKSPACE_DIR/devel/setup.bash"

# Kill existing processes
pkill -f ros || true
pkill -f gazebo || true
sleep 2

print_status "Starting Z1 Simulation..."

# Launch Gazebo
roslaunch unitree_gazebo z1.launch &
sleep 8

print_status "Gazebo ready! Starting control: $CONTROL_METHOD"

case $CONTROL_METHOD in
    keyboard|k)
        print_status "Keyboard: WASD=move, Space=open, X=close, ESC=stop"
        rosrun z1_tools z1_simple_control.py
        ;;
    xbox|x)
        roslaunch z1_tools z1_xbox_control.launch
        ;;
    demo|d)
        rosrun z1_tools z1_demo_simple.py
        ;;
    draw)
        rosrun z1_tools z1_drawing.py
        ;;
    *)
        echo "Usage: $0 [keyboard|xbox|demo|draw]"
        rosrun z1_tools z1_simple_control.py
        ;;
esac