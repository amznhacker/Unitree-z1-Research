#!/bin/bash

# Modern Z1 Launcher - Test Next-Gen Features
MODE=${1:-web}

cleanup() {
    pkill -f gazebo || true
    pkill -f roscore || true
    pkill -f rosbridge || true
    sleep 2
}

case $MODE in
    "web-modern")
        cleanup
        echo "🌐 Starting Modern Web Interface..."
        roscore &
        sleep 3
        roslaunch unitree_gazebo z1.launch &
        sleep 5
        roslaunch rosbridge_server rosbridge_websocket.launch &
        rosrun web_video_server web_video_server &
        python3 src/z1_tools/scripts/modern_web_server.py
        ;;
    "ai-vision")
        cleanup
        echo "🤖 Starting AI Vision System..."
        roscore &
        sleep 3
        roslaunch unitree_gazebo z1.launch &
        sleep 5
        python3 src/z1_tools/scripts/z1_computer_vision.py
        ;;
    "ros2-bridge")
        cleanup
        echo "🔗 Starting ROS2 Bridge (Experimental)..."
        roscore &
        sleep 3
        roslaunch unitree_gazebo z1.launch &
        sleep 5
        python3 src/z1_tools/scripts/ros2_bridge.py
        ;;
    *)
        echo "Modern Z1 Options:"
        echo "./modern_start.sh web-modern    # Next-gen web interface"
        echo "./modern_start.sh ai-vision     # AI computer vision"
        echo "./modern_start.sh ros2-bridge   # ROS2 compatibility"
        ;;
esac