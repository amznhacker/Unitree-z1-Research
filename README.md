# Unitree Z1 Robotic Arm - Complete Research Platform

**Professional 6-DOF robotic arm with simulation, real robot control, and advanced SDK integration on Ubuntu 20.04.6 LTS.**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Ubuntu 20.04](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)](https://releases.ubuntu.com/20.04/)

---

## 🚀 Quick Start (30 seconds)

```bash
git clone https://github.com/amznhacker/Unitree-z1-Research.git
cd Unitree-z1-Research
chmod +x setup_and_run.sh
./setup_and_run.sh --install-ros
```

**That's it!** The system will install ROS, build the workspace, and start the robot simulation automatically.

---

## 🎮 Usage - Unified Control System

### **Control Center (Recommended)** ⭐
```bash
./quick_start.sh web         # Opens unified Control Center
```
**Features**: Launch any script, switch simulation/real modes, system monitoring

### **Direct Control Options** 🎯
```bash
./quick_start.sh keyboard    # Direct keyboard control (WASD keys)
./quick_start.sh xbox        # Xbox controller support
./quick_start.sh gui         # Enhanced web GUI (direct)
./quick_start.sh visual      # Visual programmer (direct)
```

### **Professional Tools** 🚀
```bash
./quick_start.sh enhanced    # Full SDK integration
./quick_start.sh api         # REST API service
./quick_start.sh pure        # Pure SDK (real robot)
./quick_start.sh professional # Industrial applications
```

---

## 🎯 What You Get

### **Simulation Environment**
- **Gazebo Physics Simulation** - Realistic robot dynamics
- **6-DOF Manipulation** - Full arm + gripper control
- **Visual Feedback** - Real-time 3D visualization
- **Safe Testing** - No hardware risk

### **Control Methods**
- **Keyboard Control** - WASD + hotkeys for all joints
- **Xbox Controller** - Gamepad support for intuitive control
- **Web Interface** - Browser-based control panel
- **API Control** - HTTP REST API for remote operation
- **Voice Commands** - AI assistant integration

### **Demo Applications**
- **Pick & Place** - Object manipulation tasks
- **Drawing** - Geometric shape drawing
- **Bartender** - Cocktail mixing choreography
- **Chess Player** - Chess piece movement
- **Magic Tricks** - Entertainment performances

### **Professional Features**
- **Force Control** - Compliant manipulation
- **Trajectory Planning** - Smooth motion generation
- **Kinematics/Dynamics** - Full mathematical models
- **Real Robot Bridge** - Connect to physical hardware
- **Research Tools** - Academic research capabilities

---

## 🎯 Controls Reference

### Keyboard Control (Default)
| Key | Action | Key | Action |
|-----|--------|-----|--------|
| **W/S** | Shoulder Up/Down | **Space** | Open Gripper |
| **A/D** | Base Left/Right | **X** | Close Gripper |
| **Z/E** | Elbow Bend/Extend | **ESC** | Emergency Stop |
| **R/F** | Forearm Roll | **Q** | Quit Program |
| **T/G** | Wrist Pitch | **H** | Show Help |
| **Y/H** | Wrist Roll | | |

### Xbox Controller
- **Left Stick** - Base rotation & shoulder
- **Right Stick** - Elbow & forearm
- **D-Pad** - Wrist control
- **RT/LT** - Gripper open/close
- **Back Button** - Emergency stop

---

## 🏗️ System Architecture

### **Core Components**
```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Gazebo Sim    │◄──►│   ROS Control    │◄──►│  Z1 Hardware    │
│   (Physics)     │    │   (Middleware)   │    │  (Real Robot)   │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         ▲                        ▲                       ▲
         │                        │                       │
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│  Control Apps   │    │   Z1 SDK         │    │  Web Interface  │
│  (Python)       │    │  (C++/Python)    │    │  (Browser)      │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

### **Available Interfaces**
1. **ROS Topics** - Standard robotics communication
2. **Z1 SDK** - Direct hardware control (C++/Python)
3. **Web API** - HTTP REST interface
4. **Command Line** - Direct script execution

---

## 📁 Project Structure

```
Unitree-z1-Research/
├── 🚀 Quick Start Scripts
│   ├── setup_and_run.sh          # One-time setup + ROS installation
│   ├── quick_start.sh             # Main launcher (recommended)
│   └── system_check.sh            # Verify all components work
│
├── 🎮 Control Applications
│   ├── src/z1_tools/scripts/
│   │   ├── z1_simple_control.py   # Keyboard control
│   │   ├── z1_web_gui.py          # Browser interface
│   │   ├── z1_xbox_control.py     # Gamepad support
│   │   └── z1_visual_programmer.py # Drag-and-drop programming
│   │
├── 🎭 Demo Applications
│   │   ├── z1_demo_simple.py      # Pick & place demo
│   │   ├── z1_bartender.py        # Cocktail mixing
│   │   ├── z1_chess_player.py     # Chess playing
│   │   ├── z1_drawing.py          # Shape drawing
│   │   └── z1_magician.py         # Magic tricks
│   │
├── 🚀 Enhanced SDK Features
│   │   ├── z1_sdk_enhanced_control.py    # Full SDK integration
│   │   ├── z1_web_api_service.py         # HTTP API server
│   │   ├── z1_pure_sdk_control.py        # Production control
│   │   ├── z1_professional_suite.py      # Industrial applications
│   │   └── z1_jetson_ai_control.py       # AI-powered control
│   │
├── 🔧 Utilities
│   │   ├── z1_emergency_stop.py   # Safety stop
│   │   ├── z1_safe_limits.py      # Joint limit testing
│   │   └── z1_real_robot_bridge.py # Hardware connection
│   │
├── 🤖 Robot Definition
│   ├── src/unitree_ros/           # ROS packages
│   │   ├── robots/z1_description/ # Robot URDF/meshes
│   │   └── unitree_gazebo/        # Simulation world
│   │
├── 🧠 Z1 SDK
│   ├── src/z1_sdk/
│   │   ├── examples/              # C++ examples
│   │   ├── examples_py/           # Python examples
│   │   └── include/               # SDK headers
│   │
└── 📚 Documentation
    ├── README.md                  # This file
    ├── deployment_guide.md        # ROS vs SDK deployment
    ├── research_implementations.md # Academic research ideas
    ├── jetson_setup.md           # AI integration guide
    └── raspberry_pi_integration.md # Edge computing setup
```

---

## 🔧 Advanced Usage

### **Development Workflow**
```bash
# 1. Start Control Center
./quick_start.sh web

# 2. Develop in simulation
# Use Control Center to launch and test scripts

# 3. Switch to real robot
# Click "Real Robot Mode" in Control Center

# 4. Deploy seamlessly
# Same scripts work with real hardware
```

### **Web API Server**
```bash
# Start HTTP API server
./one_command_launcher.sh api

# Access API documentation
# Open: http://localhost:8000/docs

# Example API calls
curl -X POST http://localhost:8000/move/joint \
  -H "Content-Type: application/json" \
  -d '{"joints": [0, 0.5, -0.5, 0, 0, 0], "speed": 1.0}'
```

### **Custom Script Development**
```python
#!/usr/bin/env python3
# Template for new Z1 applications

import rospy
from std_msgs.msg import Float64

def my_custom_application():
    rospy.init_node('my_z1_app')
    
    # Publishers for joint control
    joint_pubs = {}
    for i in range(1, 7):
        topic = f"/z1_gazebo/Joint0{i}_controller/command"
        joint_pubs[f"Joint0{i}"] = rospy.Publisher(topic, Float64, queue_size=1)
    
    # Your custom logic here
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        # Move joints
        joint_pubs["Joint01"].publish(Float64(0.5))
        rate.sleep()

if __name__ == "__main__":
    my_custom_application()
```

---

## 🚀 Hardware Integration

### **Real Robot Connection**
```bash
# Method 1: Via Control Center (Recommended)
./quick_start.sh web
# Then click "Real Robot Mode" button

# Method 2: Direct SDK control
./quick_start.sh pure
```

### **Jetson AI Integration**
```bash
# AI-powered autonomous control
./one_command_launcher.sh jetson

# Requires: Jetson Xavier NX + camera
# Features: Object detection, visual servoing, autonomous manipulation
```

### **Raspberry Pi Edge Computing**
```bash
# Lightweight control on Pi
# See: raspberry_pi_integration.md
```

---

## 🔬 Research Applications

### **Academic Research Ready**
- **Trajectory Optimization** - Compare planning algorithms
- **Learning from Demonstration** - Record and replay skills
- **Force Control** - Compliant manipulation research
- **Human-Robot Collaboration** - Safety and interaction studies
- **Simulation-to-Reality Transfer** - Validate sim performance

### **Research Tools Included**
```bash
# Research implementation guides
cat research_implementations.md     # Full research roadmap
cat arm_only_research.md           # Immediate research projects
cat deployment_guide.md            # Production deployment
```

---

## 🛠️ Troubleshooting

### **Common Issues**

**Gazebo won't start:**
```bash
./fix_gazebo.sh
./system_check.sh  # Verify all components
```

**Robot not moving:**
```bash
# Check controllers are loaded
rostopic list | grep z1_gazebo
rostopic echo /z1_gazebo/joint_states
```

**Permission errors:**
```bash
sudo chown -R $USER:$USER ~/catkin_ws
chmod +x ~/catkin_ws/src/z1_tools/scripts/*.py
```

**Build errors:**
```bash
cd ~/catkin_ws
catkin_make clean
catkin_make
```

### **System Verification**
```bash
./system_check.sh  # Comprehensive system test
# Tests: ROS, Gazebo, controllers, scripts, SDK, performance
```

---

## 🎯 Capabilities Summary

### **Control Modes**
- ✅ **Manual Control** - Keyboard, gamepad, web interface
- ✅ **Programmatic Control** - Python scripts, ROS topics
- ✅ **API Control** - HTTP REST interface
- ✅ **Voice Control** - AI assistant integration
- ✅ **Visual Programming** - Drag-and-drop interface

### **Applications**
- ✅ **Pick & Place** - Object manipulation
- ✅ **Drawing** - Artistic applications
- ✅ **Assembly** - Precision manufacturing tasks
- ✅ **Entertainment** - Bartending, magic, chess
- ✅ **Research** - Academic studies and experiments

### **Deployment Options**
- ✅ **Simulation Only** - Safe development environment
- ✅ **Real Robot** - Physical hardware control
- ✅ **Hybrid** - Develop in sim, deploy to hardware
- ✅ **Edge Computing** - Raspberry Pi, Jetson integration
- ✅ **Cloud API** - Remote operation capabilities

### **Professional Features**
- ✅ **Force Control** - Compliant manipulation
- ✅ **Trajectory Planning** - Smooth motion generation
- ✅ **Safety Systems** - Emergency stops, workspace limits
- ✅ **Kinematics/Dynamics** - Full mathematical models
- ✅ **Multi-modal Control** - Position, velocity, force modes

---

## 📚 Documentation

### **Essential Guides**
| Document | Purpose |
|----------|---------|
| **README.md** | Main usage guide (this file) |
| **START_HERE.md** | Quick 3-step getting started |
| **ARCHITECTURE_GUIDE.md** | System design & critical decisions |
| **DEVELOPER_KNOWLEDGE.md** | Implementation details & patterns |
| **SYSTEM_KNOWLEDGE.md** | Complete operational guide |

### **Specialized Guides**
| Document | Purpose |
|----------|---------|
| **deployment_guide.md** | ROS vs SDK deployment strategies |
| **research_implementations.md** | Academic research opportunities |
| **jetson_setup.md** | AI integration with Jetson |
| **raspberry_pi_integration.md** | Edge computing setup |
| **arm_only_research.md** | Research with current hardware |

---

## 🤝 Contributing

1. **Fork** the repository
2. **Create** feature branch: `git checkout -b feature/amazing-feature`
3. **Commit** changes: `git commit -m 'Add amazing feature'`
4. **Push** to branch: `git push origin feature/amazing-feature`
5. **Open** Pull Request

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 🎉 Getting Started Now

**New to robotics?** Start here:
```bash
./quick_start.sh demo
```

**Want to control manually?** Try this:
```bash
./quick_start.sh keyboard
```

**Ready for advanced features?** Go with:
```bash
./leverage_everything.sh
```

**Perfect for learning robotics - safe simulation with real robot capability!**

---

## 🧠 System Knowledge

**New to the system?** Read these in order:
1. **START_HERE.md** - Quick 3-step setup
2. **README.md** - Complete feature overview (this file)
3. **ARCHITECTURE_GUIDE.md** - Understand the system design
4. **DEVELOPER_KNOWLEDGE.md** - Implementation details
5. **SYSTEM_KNOWLEDGE.md** - Operational procedures

**Key Insights:**
- **ROS vs SDK**: Use ROS for development/simulation, SDK for production
- **Script Types**: Some use `rosrun`, others use `python3` directly
- **Performance**: Simulation (50-100Hz), Real Robot (500+Hz)
- **Safety**: Always use emergency stop, understand workspace limits

---

*For questions, issues, or contributions, please visit our [GitHub repository](https://github.com/amznhacker/Unitree-z1-Research).*