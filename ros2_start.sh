#!/bin/bash

# ROS2 Z1 Launcher - Pure ROS2 Implementation
MODE=${1:-keyboard}

cleanup() {
    pkill -f gazebo || true
    pkill -f ros2 || true
    sleep 2
}

case $MODE in
    "keyboard")
        cleanup
        echo "⌨️ Starting ROS2 Keyboard Control..."
        source /opt/ros/humble/setup.bash
        cd ~/ros2_ws && colcon build
        source ~/ros2_ws/install/setup.bash
        ros2 launch z1_ros2 z1_gazebo.launch.py &
        sleep 5
        ros2 run z1_ros2 z1_keyboard_ros2.py
        ;;
    "web")
        cleanup
        echo "🌐 Starting ROS2 Web Interface..."
        source /opt/ros/humble/setup.bash
        cd ~/ros2_ws && colcon build
        source ~/ros2_ws/install/setup.bash
        ros2 launch z1_ros2 z1_gazebo.launch.py &
        sleep 5
        ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
        ros2 run z1_ros2 z1_web_ros2.py
        ;;
    "test")
        cleanup
        echo "🧪 Testing ROS2 Setup..."
        source /opt/ros/humble/setup.bash
        ros2 doctor
        ros2 pkg list | grep z1_ros2
        ;;
    *)
        echo "ROS2 Z1 Options:"
        echo "./ros2_start.sh keyboard    # ROS2 keyboard control"
        echo "./ros2_start.sh web         # ROS2 web interface"
        echo "./ros2_start.sh test        # Test ROS2 setup"
        ;;
esac