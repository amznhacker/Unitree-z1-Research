#!/bin/bash

# Fix Gazebo and Controller Issues
echo "🔧 Fixing Gazebo and controller issues..."

# Kill all gazebo processes
pkill -f gazebo
pkill -f gzserver
pkill -f gzclient
sleep 2

# Clear gazebo cache
rm -rf ~/.gazebo/log/*
rm -rf /tmp/gazebo*

# Fix controller configuration
cd ~/catkin_ws

# Install missing controller packages
sudo apt install -y \
    ros-noetic-controller-manager \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control

# Rebuild workspace
catkin_make clean
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash

echo "✅ Gazebo issues fixed. Try launching again:"
echo "./quick_start.sh keyboard"