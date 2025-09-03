#!/bin/bash

# Fix Gazebo "entity already exists" error
echo "🔧 Fixing Gazebo issues..."

# Kill all gazebo and ROS processes
pkill -f gazebo
pkill -f ros
sleep 3

# Clear gazebo cache and models
rm -rf ~/.gazebo/log/*
rm -rf /tmp/gazebo*

# Reset ROS environment
killall -9 roscore rosmaster gzserver gzclient 2>/dev/null || true
sleep 2

echo "✅ Gazebo reset complete"
echo "🚀 Now run: roslaunch unitree_gazebo z1.launch"