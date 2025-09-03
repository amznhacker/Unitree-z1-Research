#!/bin/bash

# Unitree Z1 Complete Setup Script for Ubuntu 20.04.6 LTS
# Usage: ./setup_and_run.sh [--install-ros]

set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

WORKSPACE_DIR="$HOME/catkin_ws"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_ROS=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --install-ros)
            INSTALL_ROS=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [--install-ros]"
            echo "  --install-ros: Install ROS Noetic (first time only)"
            exit 0
            ;;
        *)
            echo "Unknown option $1"
            exit 1
            ;;
    esac
done

print_status() {
    echo -e "${BLUE}[SETUP]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

install_ros_noetic() {
    print_status "Installing ROS Noetic..."
    
    sudo apt update && sudo apt upgrade -y
    
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install curl -y
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update
    sudo apt install ros-noetic-desktop-full -y
    
    if ! grep -q "source /opt/ros/noetic/setup.bash" ~/.bashrc; then
        echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    fi
    source /opt/ros/noetic/setup.bash
    
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
    
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init
    fi
    rosdep update
    
    print_success "ROS Noetic installed"
}

install_dependencies() {
    print_status "Installing dependencies..."
    
    sudo apt install -y \
        ros-noetic-controller-interface \
        ros-noetic-controller-manager \
        ros-noetic-ros-control \
        ros-noetic-ros-controllers \
        ros-noetic-gazebo-ros-control \
        ros-noetic-gazebo-ros-pkgs \
        ros-noetic-joint-state-controller \
        ros-noetic-effort-controllers \
        ros-noetic-joint-trajectory-controller \
        ros-noetic-joy \
        ros-noetic-robot-state-publisher \
        ros-noetic-xacro \
        ros-noetic-urdf \
        python3-pip \
        pybind11-dev
    
    pip3 install numpy scipy matplotlib
    
    print_success "Dependencies installed"
}

setup_workspace() {
    print_status "Setting up workspace..."
    
    mkdir -p "$WORKSPACE_DIR/src"
    
    if [ -d "$SCRIPT_DIR/src" ]; then
        cp -r "$SCRIPT_DIR/src"/* "$WORKSPACE_DIR/src/"
    fi
    
    # Fix CMakeLists.txt for catkin
    cd "$WORKSPACE_DIR"
    sudo rm -f src/CMakeLists.txt
    ln -s /opt/ros/noetic/share/catkin/cmake/toplevel.cmake src/CMakeLists.txt
    
    cd "$WORKSPACE_DIR"
    source /opt/ros/noetic/setup.bash
    
    catkin_make
    
    if ! grep -q "source $WORKSPACE_DIR/devel/setup.bash" ~/.bashrc; then
        echo "source $WORKSPACE_DIR/devel/setup.bash" >> ~/.bashrc
    fi
    source "$WORKSPACE_DIR/devel/setup.bash"
    
    print_success "Workspace built"
}

make_scripts_executable() {
    print_status "Making scripts executable..."
    
    chmod +x "$SCRIPT_DIR"/*.sh
    
    local scripts_dir="$WORKSPACE_DIR/src/z1_tools/scripts"
    if [ -d "$scripts_dir" ]; then
        chmod +x "$scripts_dir"/*.py
        chmod +x "$scripts_dir"/*.sh
    fi
    
    print_success "Scripts ready"
}

cleanup() {
    print_status "Cleaning up..."
    pkill -f ros || true
    pkill -f gazebo || true
    exit 0
}

main() {
    print_status "Unitree Z1 Setup Starting..."
    
    trap cleanup SIGINT SIGTERM
    
    if [ "$INSTALL_ROS" = true ]; then
        install_ros_noetic
    fi
    
    if ! command -v roscore &> /dev/null; then
        echo "Error: ROS not found. Run with --install-ros flag"
        exit 1
    fi
    
    install_dependencies
    setup_workspace
    make_scripts_executable
    
    print_success "Setup complete!"
    print_status "Starting Z1 simulation..."
    
    # Clear gazebo cache and launch simulation
    rm -rf ~/.gazebo/log/* 2>/dev/null || true
    rm -rf /tmp/gazebo* 2>/dev/null || true
    
    source "$WORKSPACE_DIR/devel/setup.bash"
    export GAZEBO_MODEL_PATH="$WORKSPACE_DIR/src/unitree_ros/unitree_gazebo/worlds:$GAZEBO_MODEL_PATH"
    
    roslaunch unitree_gazebo z1.launch &
    sleep 12
    
    print_success "Gazebo ready! Starting keyboard control..."
    print_status "Controls: WASD=move, Space=open, X=close, ESC=stop"
    
    rosrun z1_tools z1_simple_control.py
}

main "$@"