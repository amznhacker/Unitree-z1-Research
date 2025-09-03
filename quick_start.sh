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
killall -9 roscore rosmaster gzserver gzclient 2>/dev/null || true
sleep 5

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
        print_status "Starting Xbox controller..."
        if command -v joy_node &> /dev/null; then
            print_success "Xbox controller support detected"
            roslaunch z1_tools z1_xbox_control.launch
        else
            print_warning "Xbox controller support not installed"
            echo "Installing Xbox controller support..."
            sudo apt install -y ros-noetic-joy
            if [ $? -eq 0 ]; then
                print_success "Xbox controller support installed"
                roslaunch z1_tools z1_xbox_control.launch
            else
                echo "❌ Failed to install Xbox controller support"
                echo "💡 Run manually: sudo apt install ros-noetic-joy"
                exit 1
            fi
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
    web|w)
        print_status "Starting Web GUI..."
        print_success "Open browser: http://localhost:8080"
        rosrun z1_tools z1_web_gui.py
        ;;
    visual|v)
        print_status "Starting Visual Programmer..."
        print_success "Open browser: http://localhost:8081"
        rosrun z1_tools z1_visual_programmer.py
        ;;
    advanced|a)
        print_status "Starting Advanced Control System..."
        rosrun z1_tools z1_advanced_control.py
        ;;
    ai|voice)
        print_status "Starting AI Assistant..."
        print_success "Voice control ready - say 'Hello Z1'"
        rosrun z1_tools z1_ai_assistant.py
        ;;
    check|test)
        print_status "Running system check..."
        ./system_check.sh
        ;;
    *)
        echo "Usage: $0 [keyboard|xbox|demo|draw|real]"
        echo "  keyboard - Keyboard control (default)"
        echo "  xbox     - Xbox controller"
        echo "  demo     - Pick and place demo"
        echo "  draw     - Drawing demo"
        echo "  bartender - Cocktail mixing demo"
        echo "  web      - Web browser control"
        echo "  visual   - Visual programming (Scratch-like)"
        echo "  xbox     - Xbox controller"
        echo "  advanced - Advanced control (trajectory, force)"
        echo "  ai       - AI voice assistant"
        echo "  check    - System functionality check"
        echo "  real     - Connect to real robot"
        rosrun z1_tools z1_simple_control.py
        ;;
esac