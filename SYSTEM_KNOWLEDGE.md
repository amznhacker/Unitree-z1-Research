# System Knowledge & Operational Guide

## Complete System Capabilities

### What This System Can Do

**Simulation Capabilities:**
- ✅ **Safe Development Environment** - Test without hardware risk
- ✅ **Physics-Based Simulation** - Realistic robot dynamics in Gazebo
- ✅ **Visual Feedback** - 3D visualization of robot movements
- ✅ **Controller Testing** - Validate algorithms before real deployment

**Real Robot Capabilities:**
- ✅ **Direct Hardware Control** - Sub-millisecond response times
- ✅ **Professional Precision** - Industrial-grade manipulation
- ✅ **Force Control** - Compliant interaction with environment
- ✅ **High-Speed Operation** - 500+ Hz control loops

**AI Integration Ready:**
- ✅ **Jetson Xavier NX** - Real-time computer vision (30+ FPS)
- ✅ **Object Detection** - Autonomous pick and place
- ✅ **Visual Servoing** - Track and follow objects
- ✅ **Edge Computing** - All processing on-device

**Research Applications:**
- ✅ **Trajectory Optimization** - Compare planning algorithms
- ✅ **Learning from Demonstration** - Record and replay skills
- ✅ **Human-Robot Collaboration** - Safe interaction studies
- ✅ **Simulation-to-Reality Transfer** - Validate algorithms

## System Execution Modes

### Mode 1: Simulation Development
```bash
# Complete ROS environment with Gazebo
./quick_start.sh keyboard    # Safe learning environment
./quick_start.sh demo        # Algorithm testing
./quick_start.sh bartender   # Complex choreography
```
**Use For**: Learning, development, algorithm testing, demonstrations

### Mode 2: Real Robot Production  
```bash
# Direct SDK control, maximum performance
python3 z1_pure_sdk_control.py      # Direct hardware control
python3 z1_professional_suite.py    # Industrial applications
```
**Use For**: Production deployment, research data collection, real applications

### Mode 3: Hybrid Development
```bash
# Develop in simulation, deploy to real robot
./quick_start.sh enhanced    # ROS + SDK bridge
./quick_start.sh real        # Simulation bridge to hardware
```
**Use For**: Algorithm development with real robot validation

### Mode 4: Web/API Control
```bash
# Remote control via HTTP API
./quick_start.sh api         # Start web service
curl -X POST localhost:8000/move/joint -d '{"joints":[0,0.5,0,0,0,0]}'
```
**Use For**: Remote operation, integration with other systems, web interfaces

## Hardware Integration Options

### Option 1: PC Only (Simulation)
```
PC (Ubuntu 20.04) → ROS + Gazebo → Virtual Z1
```
**Capabilities**: Safe development, algorithm testing, learning
**Limitations**: No real robot interaction

### Option 2: PC + Real Robot
```
PC (Ubuntu 20.04) → Z1 SDK → Physical Z1 Robot
```
**Capabilities**: Full robot control, research applications, production use
**Requirements**: Physical Z1 robot, network connection

### Option 3: PC + Jetson + Real Robot
```
PC (Development) ↔ Jetson Xavier NX (AI Vision) → Z1 Robot
```
**Capabilities**: Autonomous manipulation, computer vision, AI applications
**Requirements**: Jetson Xavier NX, camera, Z1 robot

### Option 4: Raspberry Pi + Real Robot
```
Raspberry Pi 4 → Edge Computing + Sensors → Z1 Robot
```
**Capabilities**: Distributed control, sensor integration, mobile platforms
**Requirements**: Raspberry Pi 4, sensors, Z1 robot

## Network Architecture

### Local Development Network
```
PC: 192.168.1.100
├── ROS Master: localhost:11311
├── Gazebo: localhost:11345  
└── Web API: localhost:8000
```

### Real Robot Network
```
PC: 192.168.123.100
Z1 Robot: 192.168.123.110
├── SDK Communication: UDP/TCP
├── Control Frequency: 500Hz
└── Latency: <1ms
```

### Distributed System Network
```
PC: 192.168.1.100 (Planning)
Jetson: 192.168.1.101 (AI Vision)
Pi: 192.168.1.102 (Sensors)
Z1: 192.168.123.110 (Robot)
```

## Performance Characteristics

### Simulation Performance
| Metric | Value | Notes |
|--------|-------|-------|
| Control Frequency | 50-100 Hz | Limited by Gazebo physics |
| Latency | 10-50ms | ROS message passing overhead |
| CPU Usage | 30-60% | Depends on Gazebo complexity |
| Memory Usage | 2-4GB | ROS + Gazebo + controllers |

### Real Robot Performance  
| Metric | Value | Notes |
|--------|-------|-------|
| Control Frequency | 500+ Hz | Direct SDK communication |
| Latency | <1ms | No middleware overhead |
| CPU Usage | 5-15% | Efficient SDK implementation |
| Memory Usage | 100-500MB | Minimal overhead |

### AI Integration Performance (Jetson Xavier NX)
| Metric | Value | Notes |
|--------|-------|-------|
| Object Detection | 30+ FPS | Real-time YOLO/SSD |
| Inference Time | 10-20ms | Edge AI processing |
| Power Usage | 15W | Efficient edge computing |
| Memory Usage | 2-4GB | AI models + robot control |

## Safety Systems

### Software Safety Features
- **Joint Limit Enforcement** - Prevents mechanical damage
- **Workspace Boundaries** - Keeps robot in safe operating area
- **Emergency Stop** - Immediate halt via ESC key or API
- **Force Monitoring** - Detects excessive forces/collisions
- **Velocity Limiting** - Prevents dangerous high-speed movements

### Hardware Safety Requirements
- **Physical Emergency Stop** - Hardware button for real robot
- **Proper Mounting** - Secure robot base installation
- **Clear Workspace** - Remove obstacles and hazards
- **Network Security** - Secure robot network access
- **Power Management** - Proper electrical installation

## Troubleshooting Decision Tree

### Problem: Robot Not Moving
1. **Check Simulation**: Is Gazebo running? `rostopic list | grep z1_gazebo`
2. **Check Controllers**: Are joint controllers loaded? `rostopic echo /z1_gazebo/joint_states`
3. **Check Commands**: Are commands being published? `rostopic echo /z1_gazebo/Joint01_controller/command`
4. **Check Real Robot**: Is robot connected? `ping 192.168.123.110`

### Problem: Scripts Won't Start
1. **Check ROS Environment**: `source ~/catkin_ws/devel/setup.bash`
2. **Check Permissions**: `chmod +x ~/catkin_ws/src/z1_tools/scripts/*.py`
3. **Check Dependencies**: `python3 -c "import rospy, numpy, flask"`
4. **Check Workspace**: `ls ~/catkin_ws/devel/setup.bash`

### Problem: Poor Performance
1. **Simulation**: Use headless Gazebo `gui:=false`
2. **Clear Cache**: `rm -rf ~/.gazebo/log/*`
3. **Check Resources**: `htop` - monitor CPU/memory
4. **Network**: Check latency `ping 192.168.123.110`

## Maintenance Procedures

### Daily Operations
```bash
# Start system
./quick_start.sh keyboard

# Check system health  
./system_check.sh

# Clean shutdown
# Press ESC in control script, then Ctrl+C
```

### Weekly Maintenance
```bash
# Clear Gazebo cache
rm -rf ~/.gazebo/log/*
rm -rf /tmp/gazebo*

# Update system
sudo apt update && sudo apt upgrade

# Check disk space
df -h ~/catkin_ws
```

### Monthly Maintenance
```bash
# Rebuild workspace
cd ~/catkin_ws
catkin_make clean
catkin_make

# Check for updates
git pull origin main

# Verify all components
./system_check.sh
```

## Backup & Recovery

### Critical Files to Backup
```
~/catkin_ws/src/z1_tools/scripts/    # Custom control scripts
~/.bashrc                            # Environment setup
~/catkin_ws/devel/setup.bash        # Workspace configuration
```

### Recovery Procedure
```bash
# Complete system recovery
git clone https://github.com/amznhacker/Unitree-z1-Research.git
cd Unitree-z1-Research
./setup_and_run.sh --install-ros

# Restore custom scripts
cp backup/scripts/* ~/catkin_ws/src/z1_tools/scripts/
```

## Deployment Scenarios

### Scenario 1: Educational Lab
- **Setup**: Multiple PCs with simulation
- **Use**: Student learning, safe experimentation
- **Scripts**: `./quick_start.sh keyboard`, `./quick_start.sh demo`

### Scenario 2: Research Lab
- **Setup**: PC + Real Robot + Jetson
- **Use**: Algorithm development, data collection
- **Scripts**: `./quick_start.sh enhanced`, `python3 z1_pure_sdk_control.py`

### Scenario 3: Industrial Application
- **Setup**: Embedded PC + Real Robot
- **Use**: Production automation, quality control
- **Scripts**: `python3 z1_professional_suite.py`

### Scenario 4: Mobile Platform
- **Setup**: Raspberry Pi + Real Robot + Battery
- **Use**: Mobile manipulation, field robotics
- **Scripts**: Custom Pi-optimized control scripts

This system knowledge provides the complete operational understanding needed to effectively use and maintain the Z1 robotic system.