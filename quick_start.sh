#!/bin/bash

# Unitree Z1 Quick Start - Fixed for Ubuntu 20.04.6 LTS
# Usage: ./quick_start.sh [keyboard|xbox|demo|draw|real]

WORKSPACE_DIR="$HOME/catkin_ws"
CONTROL_METHOD="${1:-keyboard}"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_status() {
    echo -e "${BLUE}[Z1]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
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
print_status "Cleaning up existing processes..."
pkill -f ros || true
pkill -f gazebo || true
sleep 3

# Clear gazebo cache
print_status "Clearing Gazebo cache..."
rm -rf ~/.gazebo/log/* 2>/dev/null || true
rm -rf /tmp/gazebo* 2>/dev/null || true

# Set environment
export GAZEBO_MODEL_PATH="$WORKSPACE_DIR/src/unitree_ros/unitree_gazebo/worlds:$GAZEBO_MODEL_PATH"

print_status "Starting Z1 Simulation..."

# Launch Gazebo
roslaunch unitree_gazebo z1.launch &
GAZEBO_PID=$!

# Wait for Gazebo to be ready
print_status "Waiting for Gazebo to initialize..."
sleep 15

# Check if Gazebo is running
if ! ps -p $GAZEBO_PID > /dev/null; then
    echo "Error: Gazebo failed to start"
    exit 1
fi

# Wait for controllers
print_status "Waiting for controllers to load..."
timeout=30
count=0
while [ $count -lt $timeout ]; do
    if rostopic list 2>/dev/null | grep -q "/z1_gazebo/joint_states"; then
        print_success "Controllers loaded successfully"
        break
    fi
    sleep 1
    ((count++))
done

if [ $count -eq $timeout ]; then
    print_warning "Controllers may not be fully loaded, continuing anyway..."
fi

print_success "Gazebo ready! Starting control: $CONTROL_METHOD"

case $CONTROL_METHOD in
    keyboard|k)
        print_status "Keyboard: WASD=move, ZE=elbow, Space=open, X=close, ESC=stop"
        rosrun z1_tools z1_simple_control.py
        ;;
    xbox|x)
        if command -v joy_node &> /dev/null; then
            roslaunch z1_tools z1_xbox_control.launch
        else
            echo "Error: joy_node not found. Install with: sudo apt install ros-noetic-joy"
            exit 1
        fi
        ;;
    demo|d)
        rosrun z1_tools z1_demo_simple.py
        ;;
    draw)
        rosrun z1_tools z1_drawing.py
        ;;
    bartender|b)
        rosrun z1_tools z1_bartender.py
        ;;
    real|r)
        print_warning "REAL ROBOT MODE - Ensure robot is connected and safe"
        print_status "Starting simulation bridge to real robot..."
        rosrun z1_tools z1_real_robot_bridge.py &
        sleep 2
        rosrun z1_tools z1_simple_control.py
        ;;
    *)
        echo "Usage: $0 [keyboard|xbox|demo|draw|real]"
        echo "  keyboard - Keyboard control (default)"
        echo "  xbox     - Xbox controller"
        echo "  demo     - Pick and place demo"
        echo "  draw     - Drawing demo"
        echo "  bartender - Cocktail mixing demo"
        echo "  real     - Connect to real robot"
        rosrun z1_tools z1_simple_control.py
        ;;
esac