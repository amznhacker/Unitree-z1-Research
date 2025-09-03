# Unitree Z1 Setup Instructions for Ubuntu 20.04.6 LTS

## Prerequisites
- Ubuntu 20.04.6 LTS
- Internet connection
- At least 4GB RAM and 20GB free disk space

## 1. Install ROS Noetic

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS Noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full -y

# Setup ROS environment
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install ROS dependencies
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
sudo rosdep init
rosdep update
```

## 2. Install Required ROS Packages

```bash
sudo apt install -y \
    ros-noetic-controller-interface \
    ros-noetic-gazebo-ros-control \
    ros-noetic-joint-state-controller \
    ros-noetic-effort-controllers \
    ros-noetic-joint-trajectory-controller \
    ros-noetic-joy \
    ros-noetic-robot-state-publisher \
    ros-noetic-xacro \
    ros-noetic-urdf
```

## 3. Setup Workspace

```bash
# Create workspace (if not already exists)
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws

# Copy your unitree_ws contents to catkin_ws
# If you have the files locally, copy them:
# cp -r /path/to/unitree_ws/src/* ~/catkin_ws/src/

# Build workspace
catkin_make

# Source workspace
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/catkin_ws/devel/setup.bash
```

## 4. Install Additional Dependencies

```bash
# Install Python dependencies
sudo apt install python3-pip -y
pip3 install numpy scipy matplotlib

# Install pybind11 for Z1 SDK Python bindings
sudo apt install pybind11-dev -y

# Make scripts executable
cd ~/catkin_ws/src/z1_tools/scripts
chmod +x *.py
chmod +x *.sh
```

## 5. Configure Gazebo Paths

```bash
# Edit stairs world file
nano ~/catkin_ws/src/unitree_ros/unitree_gazebo/worlds/stairs.world

# Change the path at the end of file from:
# <uri>model:///home/unitree/catkin_ws/src/unitree_ros/unitree_gazebo/worlds/building_editor_models/stairs</uri>
# To:
# <uri>model://$(find unitree_gazebo)/worlds/building_editor_models/stairs</uri>
```

## 6. Build Z1 SDK

```bash
cd ~/catkin_ws/src/z1_sdk
mkdir build && cd build
cmake ..
make -j4
```

## 7. Test Installation

### Launch Z1 Simulation:
```bash
# Terminal 1: Launch Gazebo with Z1
roslaunch unitree_gazebo z1.launch

# Terminal 2: Test keyboard control
rosrun z1_tools z1_keyboard_control.py
```

### Test Xbox Controller (Optional):
```bash
# Setup Xbox controller
cd ~/catkin_ws/src/z1_tools/scripts
./setup_xbox_controller.sh

# Launch with Xbox control
roslaunch z1_tools z1_xbox_control.launch
```

## 8. Available Commands

### Basic Control:
```bash
# Keyboard control
rosrun z1_tools z1_keyboard_control.py

# Pick and place demo
rosrun z1_tools z1_pick_place.py

# Drawing demo
rosrun z1_tools z1_drawing.py

# Emergency stop
rosrun z1_tools z1_emergency_stop.py
```

### Entertainment Demos:
```bash
# Chess player
rosrun z1_tools z1_chess_player.py

# Bartender
rosrun z1_tools z1_bartender.py

# Musician
rosrun z1_tools z1_musician.py

# Magician
rosrun z1_tools z1_magician.py
```

## 9. Troubleshooting

### If catkin_make fails:
```bash
# Clean and rebuild
cd ~/catkin_ws
catkin_make clean
catkin_make
```

### If Gazebo doesn't start:
```bash
# Reset Gazebo
killall gzserver gzclient
roslaunch unitree_gazebo z1.launch
```

### If joints don't move:
```bash
# Check if controllers are loaded
rostopic list | grep controller
rosservice call /controller_manager/list_controllers
```

### Permission issues:
```bash
# Fix script permissions
cd ~/catkin_ws/src/z1_tools/scripts
chmod +x *.py
sudo chown -R $USER:$USER ~/catkin_ws
```

## 10. System Requirements Check

```bash
# Check ROS installation
roscore &
sleep 2
rostopic list
killall roscore

# Check Gazebo
gazebo --version

# Check Python packages
python3 -c "import numpy, scipy, matplotlib; print('Python packages OK')"
```

## Quick Start Commands

```bash
# 1. Launch simulation
roslaunch unitree_gazebo z1.launch

# 2. In new terminal - start control
rosrun z1_tools z1_keyboard_control.py

# 3. Or run a demo
rosrun z1_tools z1_pick_place.py
```

## Notes

- Always source the workspace: `source ~/catkin_ws/devel/setup.bash`
- Use `roscore` if needed before launching nodes
- Press `Ctrl+C` to stop any running script
- Use `ESC` key for emergency stop in most scripts
- Gazebo may take time to load initially

## Safety Reminders

⚠️ **For Simulation Only**: These instructions are for Gazebo simulation. Real hardware requires additional safety measures and different parameters.