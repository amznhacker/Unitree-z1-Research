#!/bin/bash

# Modern Z1 Setup - Next Generation Features
# Ubuntu 20.04.6 LTS with modern stack preparation

set -e

echo "🚀 Setting up Modern Z1 Stack..."

# Install modern Python and Node.js
sudo apt update
sudo apt install -y python3.9 python3.9-pip nodejs npm

# Modern web stack
npm install -g @angular/cli express socket.io

# AI/ML stack
pip3 install torch torchvision ultralytics opencv-python-headless
pip3 install transformers speechrecognition pyttsx3

# Modern ROS tools
sudo apt install -y ros-noetic-rosbridge-server ros-noetic-web-video-server
sudo apt install -y ros-noetic-tf2-web-republisher

# Docker for containerization
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER

# Build workspace with modern features
cd ~/catkin_ws
catkin_make

echo "✅ Modern stack ready! Use ./modern_start.sh to test features"