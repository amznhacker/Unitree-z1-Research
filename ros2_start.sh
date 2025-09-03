#!/bin/bash

# ROS2 Z1 Launcher - Complete Modern Implementation
MODE=${1:-help}

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

cleanup() {
    echo -e "${YELLOW}🧹 Cleaning up processes...${NC}"
    pkill -f gazebo || true
    pkill -f ros2 || true
    pkill -f rviz2 || true
    pkill -f controller_manager || true
    sleep 3
    echo -e "${GREEN}✅ Cleanup complete${NC}"
}

check_ros2() {
    if ! command -v ros2 &> /dev/null; then
        echo -e "${RED}❌ ROS2 not found. Run ./ros2_setup.sh first${NC}"
        exit 1
    fi
}

build_workspace() {
    echo -e "${BLUE}🔨 Building workspace...${NC}"
    cd ~/ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    source ~/ros2_ws/install/setup.bash
    echo -e "${GREEN}✅ Build complete${NC}"
}

case $MODE in
    "keyboard")
        cleanup
        check_ros2
        echo -e "${BLUE}⌨️ Starting ROS2 Keyboard Control...${NC}"
        build_workspace
        
        # Start Gazebo simulation
        echo -e "${YELLOW}🚀 Launching Gazebo...${NC}"
        ros2 launch z1_ros2 z1_gazebo.launch.py &
        sleep 8
        
        # Start controllers
        echo -e "${YELLOW}🎮 Starting controllers...${NC}"
        ros2 control load_controller joint_state_broadcaster
        ros2 control load_controller z1_arm_controller
        ros2 control set_controller_state joint_state_broadcaster active
        ros2 control set_controller_state z1_arm_controller active
        sleep 2
        
        # Start keyboard control
        echo -e "${GREEN}✅ Ready for keyboard control${NC}"
        ros2 run z1_ros2 z1_keyboard_ros2.py
        ;;
        
    "web")
        cleanup
        check_ros2
        echo -e "${BLUE}🌐 Starting ROS2 Web Interface...${NC}"
        build_workspace
        
        # Start Gazebo simulation
        echo -e "${YELLOW}🚀 Launching Gazebo...${NC}"
        ros2 launch z1_ros2 z1_gazebo.launch.py &
        sleep 8
        
        # Start controllers
        echo -e "${YELLOW}🎮 Starting controllers...${NC}"
        ros2 control load_controller joint_state_broadcaster
        ros2 control load_controller z1_arm_controller
        ros2 control set_controller_state joint_state_broadcaster active
        ros2 control set_controller_state z1_arm_controller active
        sleep 2
        
        # Start rosbridge
        echo -e "${YELLOW}🌉 Starting ROS bridge...${NC}"
        ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
        sleep 3
        
        # Start web interface
        echo -e "${GREEN}🌐 Web interface starting at http://localhost:8080${NC}"
        ros2 run z1_ros2 z1_web_ros2.py
        ;;
        
    "rviz")
        cleanup
        check_ros2
        echo -e "${BLUE}👁️ Starting ROS2 with RViz...${NC}"
        build_workspace
        
        # Start simulation and RViz
        ros2 launch z1_ros2 z1_gazebo.launch.py &
        sleep 5
        rviz2 -d ~/ros2_ws/src/z1_ros2/config/z1.rviz &
        
        echo -e "${GREEN}✅ RViz and simulation ready${NC}"
        read -p "Press Enter to exit..."
        ;;
        
    "test")
        echo -e "${BLUE}🧪 Testing ROS2 Setup...${NC}"
        check_ros2
        
        echo -e "${YELLOW}📋 ROS2 Doctor Report:${NC}"
        ros2 doctor
        
        echo -e "\n${YELLOW}📦 Available Packages:${NC}"
        ros2 pkg list | grep -E "(z1|control|gazebo)" || echo "No Z1 packages found"
        
        echo -e "\n${YELLOW}🎮 Available Controllers:${NC}"
        ros2 control list_controllers 2>/dev/null || echo "No controllers running"
        
        echo -e "\n${YELLOW}📡 Active Topics:${NC}"
        ros2 topic list | head -10
        
        echo -e "\n${GREEN}✅ Test complete${NC}"
        ;;
        
    "build")
        echo -e "${BLUE}🔨 Building Z1 ROS2 Package...${NC}"
        check_ros2
        build_workspace
        ;;
        
    "clean")
        echo -e "${YELLOW}🧹 Cleaning build files...${NC}"
        cd ~/ros2_ws
        rm -rf build/ install/ log/
        echo -e "${GREEN}✅ Clean complete${NC}"
        ;;
        
    "help"|*)
        echo -e "${BLUE}🤖 Z1 ROS2 Control Options:${NC}"
        echo -e "${GREEN}./ros2_start.sh keyboard${NC}    # Keyboard control interface"
        echo -e "${GREEN}./ros2_start.sh web${NC}         # Modern web interface"
        echo -e "${GREEN}./ros2_start.sh rviz${NC}        # RViz visualization"
        echo -e "${GREEN}./ros2_start.sh test${NC}        # System diagnostics"
        echo -e "${GREEN}./ros2_start.sh build${NC}       # Build workspace"
        echo -e "${GREEN}./ros2_start.sh clean${NC}       # Clean build files"
        echo ""
        echo -e "${YELLOW}📋 Prerequisites:${NC}"
        echo -e "   1. Run ${BLUE}./ros2_setup.sh${NC} first"
        echo -e "   2. Restart terminal after setup"
        echo -e "   3. Ensure Z1 package is in ~/ros2_ws/src/"
        ;;
esac