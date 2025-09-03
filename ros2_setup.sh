#!/bin/bash

# ROS2 Z1 Setup - Complete Modern Stack
# Ubuntu 22.04+ LTS with ROS2 Humble/Iron

set -e

echo "🚀 Setting up Complete ROS2 Z1 Stack..."

# System update
sudo apt update && sudo apt upgrade -y

# Install ROS2 Humble
echo "📦 Installing ROS2 Humble..."
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install -y curl gnupg lsb-release

# Add ROS2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions

# ROS2 Control and Gazebo
echo "🎮 Installing ROS2 Control..."
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install -y ros-humble-gazebo-ros2-control ros-humble-gazebo-ros-pkgs
sudo apt install -y ros-humble-joint-state-publisher ros-humble-robot-state-publisher
sudo apt install -y ros-humble-xacro ros-humble-joint-trajectory-controller
sudo apt install -y ros-humble-position-controllers ros-humble-velocity-controllers

# Web and networking
echo "🌐 Installing Web Stack..."
sudo apt install -y ros-humble-rosbridge-suite ros-humble-web-video-server
sudo apt install -y ros-humble-tf2-web-republisher

# Modern development tools
echo "🛠️ Installing Development Tools..."
sudo apt install -y python3-pip python3-venv nodejs npm git
npm install -g @angular/cli express socket.io

# Python packages
echo "🐍 Installing Python Packages..."
pip3 install --user flask flask-socketio
pip3 install --user torch torchvision ultralytics opencv-python-headless
pip3 install --user transformers speechrecognition pyttsx3
pip3 install --user numpy scipy matplotlib

# Docker (optional)
echo "🐳 Installing Docker..."
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
rm get-docker.sh

# Environment setup
echo "⚙️ Setting up Environment..."
echo "# ROS2 Environment" >> ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:~/ros2_ws/src/z1_ros2/models" >> ~/.bashrc

# Create workspace
echo "📁 Creating ROS2 Workspace..."
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Copy Z1 package to workspace
if [ -d "$(pwd)/src/z1_ros2" ]; then
    echo "Z1 package found, building..."
else
    echo "Please copy z1_ros2 package to ~/ros2_ws/src/"
fi

# Build workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Source workspace
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

echo "✅ ROS2 Z1 Stack Installation Complete!"
echo "📋 Next Steps:"
echo "   1. Restart terminal or run: source ~/.bashrc"
echo "   2. Test installation: ./ros2_start.sh test"
echo "   3. Start simulation: ./ros2_start.sh keyboard"
echo "   4. Web interface: ./ros2_start.sh web"
echo "🌐 Web interface will be at: http://localhost:8080"