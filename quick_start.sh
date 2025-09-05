#!/bin/bash

# Unitree Z1 Quick Start - Fixed for Ubuntu 20.04.6 LTS
# Usage: ./quick_start.sh [keyboard|xbox|demo|draw|real]

WORKSPACE_DIR="$HOME/catkin_ws"
CONTROL_METHOD="${1:-keyboard}"

# Show help if no arguments
if [ $# -eq 0 ]; then
    echo "Unitree Z1 Quick Start"
    echo "Usage: $0 [option]"
    echo "Run '$0 list' for all options, or '$0 keyboard' to start"
    echo ""
    CONTROL_METHOD="keyboard"
fi

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
        # Check if Xbox controller is connected
        if [ ! -e /dev/input/js0 ]; then
            print_warning "No Xbox controller detected at /dev/input/js0"
            echo "💡 Connect Xbox controller via USB or Bluetooth"
            echo "💡 Check available controllers: ls /dev/input/js*"
            read -p "Continue anyway? (y/n): " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                exit 1
            fi
        fi
        
        # Install joy package if needed
        if ! dpkg -l | grep -q ros-noetic-joy; then
            print_status "Installing Xbox controller support..."
            sudo apt update && sudo apt install -y ros-noetic-joy
            if [ $? -ne 0 ]; then
                echo "❌ Failed to install ros-noetic-joy"
                echo "💡 Run manually: sudo apt install ros-noetic-joy"
                exit 1
            fi
        fi
        
        print_success "Starting Xbox controller interface..."
        print_status "Controls: Left stick=base/shoulder, Right stick=elbow/forearm, D-pad=wrist, RT/LT=gripper"
        roslaunch z1_tools z1_xbox_control.launch
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
        print_status "Starting Z1 Control Center..."
        print_success "Open browser: http://localhost:8080"
        print_status "Features: Unified script launcher, simulation/real switching, system monitoring"
        rosrun z1_tools z1_control_center.py
        ;;
    visual|v)
        print_status "Starting Enhanced Visual Programmer..."
        print_success "Open browser: http://localhost:8081"
        print_status "Features: 3D preview, simulation, advanced blocks, drag-and-drop programming"
        rosrun z1_tools z1_visual_programmer_enhanced.py
        ;;
    gui|g)
        print_status "Starting Enhanced Web GUI (Direct)..."
        print_success "Open browser: http://localhost:8080"
        print_status "Features: Real-time 3D visualization, mobile-friendly interface, trajectory control"
        rosrun z1_tools z1_web_gui_enhanced.py
        ;;
    programmer|p)
        print_status "Starting Visual Programmer (Direct)..."
        print_success "Open browser: http://localhost:8081"
        print_status "Features: 3D preview, simulation, advanced blocks, drag-and-drop programming"
        rosrun z1_tools z1_visual_programmer_enhanced.py
        ;;
    enhanced|e)
        print_status "Starting Enhanced SDK Control..."
        rosrun z1_tools z1_sdk_enhanced_control.py
        ;;
    api)
        print_status "Starting Web API Service..."
        print_success "API Documentation: http://localhost:8000/docs"
        rosrun z1_tools z1_web_api_service.py
        ;;
    pure|p)
        print_status "Starting Pure SDK Control (No ROS)..."
        python3 src/z1_tools/scripts/z1_pure_sdk_control.py
        ;;
    professional|pro)
        print_status "Starting Professional Suite..."
        rosrun z1_tools z1_professional_suite.py
        ;;
    jetson|ai)
        print_status "Starting Jetson AI Control..."
        rosrun z1_tools z1_jetson_ai_control.py
        ;;
    chess|c)
        print_status "Starting Chess Player..."
        rosrun z1_tools z1_chess_player.py
        ;;
    magic|m)
        print_status "Starting Magic Tricks..."
        rosrun z1_tools z1_magician.py
        ;;
    wave)
        print_status "Starting Wave Demo..."
        rosrun z1_tools z1_wave.py
        ;;
    music)
        print_status "Starting Musical Movements..."
        rosrun z1_tools z1_musician.py
        ;;
    painter)
        print_status "Starting Artistic Painter..."
        rosrun z1_tools z1_painter.py
        ;;
    dancer)
        print_status "Starting Dance Choreography..."
        rosrun z1_tools z1_dancer.py
        ;;
    juggler)
        print_status "Starting Juggling Performance..."
        rosrun z1_tools z1_juggler.py
        ;;
    chef)
        print_status "Starting Cooking Demo..."
        rosrun z1_tools z1_chef.py
        ;;
    drummer)
        print_status "Starting Drum Performance..."
        rosrun z1_tools z1_drummer.py
        ;;
    yoga)
        print_status "Starting Yoga Instructor..."
        rosrun z1_tools z1_yoga_instructor.py
        ;;
    mime)
        print_status "Starting Mime Artist..."
        rosrun z1_tools z1_mime_artist.py
        ;;
    check|test)
        print_status "Running system check..."
        ./system_check.sh
        ;;
    list|help|-h|--help)
        echo "Unitree Z1 Quick Start - All Available Options"
        echo ""
        echo "🎮 Basic Control:"
        echo "  keyboard  - Keyboard control (WASD keys)"
        echo "  xbox      - Xbox controller support"
        echo "  web       - Control Center (unified interface)"
        echo "  visual    - Visual programming (drag-and-drop)"
        echo ""
        echo "🎯 Demos:"
        echo "  demo      - Pick and place demonstration"
        echo "  draw      - Drawing geometric shapes"
        echo "  bartender - Cocktail mixing choreography"
        echo "  chess     - Chess playing moves"
        echo "  magic     - Magic trick performances"
        echo "  wave      - Simple wave motions"
        echo "  music     - Musical conducting"
        echo "  painter   - Artistic painting"
        echo "  dancer    - Dance choreography"
        echo "  juggler   - Juggling performance"
        echo "  chef      - Cooking demonstration"
        echo "  drummer   - Drum performance"
        echo "  yoga      - Yoga instruction"
        echo "  mime      - Mime artist performance"
        echo ""
        echo "🚀 Enhanced Features:"
        echo "  enhanced  - Full SDK integration"
        echo "  api       - Web API service"
        echo "  pure      - Pure SDK (no ROS)"
        echo "  professional - Industrial applications"
        echo "  jetson    - AI-powered control"
        echo "  gui       - Enhanced Web GUI (direct)"
        echo "  programmer - Visual Programmer (direct)"
        echo ""
        echo "🔧 Utilities:"
        echo "  real      - Connect to real robot"
        echo "  check     - System functionality test"
        echo "  list      - Show this help"
        echo ""
        echo "Usage: $0 [option]"
        echo "Example: $0 bartender"
        exit 0
        ;;
    *)
        echo "Unknown option: $CONTROL_METHOD"
        echo "Run '$0 list' to see all available options"
        echo "Starting default keyboard control..."
        rosrun z1_tools z1_simple_control.py
        ;;
esac