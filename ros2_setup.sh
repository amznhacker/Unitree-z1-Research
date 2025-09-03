#!/bin/bash

# ROS2 Z1 Setup - Full Port to ROS2 Humble/Iron
# Ubuntu 22.04+ LTS

set -e

echo "🚀 Setting up ROS2 Z1 Stack..."

# Install ROS2 Humble
sudo apt update
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions

# ROS2 development tools
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-joint-state-publisher
sudo apt install -y ros-humble-robot-state-publisher ros-humble-xacro

# Modern Python stack
sudo apt install -y python3-pip
pip3 install torch torchvision ultralytics opencv-python

# Web interface
sudo apt install -y ros-humble-rosbridge-suite ros-humble-web-video-server

# Source ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash

# Create ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build

echo "✅ ROS2 setup complete! Use ./ros2_start.sh to test"