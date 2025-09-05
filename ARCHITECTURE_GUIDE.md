# Z1 System Architecture & Design Decisions

## System Overview

The Unitree Z1 research platform uses a **hybrid architecture** combining ROS for development/simulation and pure SDK for production deployment.

```
Development Stack:    ROS + Gazebo + SDK Bridge → Safe Learning
Production Stack:     Pure SDK → Real Robot → Maximum Performance
```

## Critical Design Decisions

### 1. **ROS vs Pure SDK Strategy**

**Why We Use Both:**
- **ROS**: Required for Gazebo simulation, provides safety and visualization
- **Pure SDK**: Required for real-time performance and production deployment
- **Hybrid**: Best of both worlds - develop safely, deploy efficiently

**Script Classification:**
```bash
# ROS-Dependent Scripts (need rosrun)
z1_simple_control.py      # Uses ROS topics for simulation
z1_demo_simple.py         # Publishes to joint controllers
z1_bartender.py           # ROS-based choreography

# Pure SDK Scripts (use python3 directly)  
z1_pure_sdk_control.py    # Direct hardware communication
z1_professional_suite.py  # Industrial applications
z1_web_api_service.py     # HTTP API server
z1_jetson_ai_control.py   # AI-powered control
```

### 2. **Workspace Structure Logic**

```
~/catkin_ws/                    # ROS workspace (required for simulation)
├── src/
│   ├── unitree_ros/           # Robot definition & Gazebo
│   ├── z1_sdk/                # Direct hardware SDK
│   └── z1_tools/              # Our control applications
│
Project Root/                   # Git repository
├── setup_and_run.sh          # One-time system setup
├── quick_start.sh             # Universal launcher
└── *.md                       # Documentation
```

**Why This Structure:**
- ROS requires catkin workspace for simulation
- SDK can run independently for real robot
- Git repo contains launchers and documentation
- Separation allows flexible deployment

### 3. **Control Flow Architecture**

```
User Command → quick_start.sh → Case Statement → Appropriate Execution Method
                                      ↓
                              ROS Script: rosrun z1_tools script.py
                                      ↓
                              Pure SDK: python3 path/to/script.py
```

## Performance Characteristics

### Simulation (ROS + Gazebo)
- **Control Frequency**: 50-100 Hz
- **Latency**: 10-50ms
- **Use Case**: Development, learning, testing
- **Safety**: Complete (no hardware risk)

### Real Robot (Pure SDK)
- **Control Frequency**: 500+ Hz  
- **Latency**: <1ms
- **Use Case**: Production, research, performance
- **Safety**: Requires careful programming

## Script Execution Methods

### Method 1: ROS Scripts
```bash
# Requires ROS environment and Gazebo running
source ~/catkin_ws/devel/setup.bash
roslaunch unitree_gazebo z1.launch    # Start simulation
rosrun z1_tools z1_simple_control.py  # Run script
```

### Method 2: Pure SDK Scripts  
```bash
# Direct execution, no ROS needed
python3 ~/catkin_ws/src/z1_tools/scripts/z1_pure_sdk_control.py
```

### Method 3: Unified Launcher (Recommended)
```bash
# Automatically handles ROS vs SDK execution
./quick_start.sh keyboard    # ROS script
./quick_start.sh pure        # SDK script
```

## Hardware Integration Paths

### Simulation Only
```
PC → ROS + Gazebo → Virtual Z1 → Safe Learning
```

### Real Robot
```
PC → Z1 SDK → Physical Z1 → Production Control
```

### Jetson Integration
```
Jetson Xavier NX → AI Vision + Z1 SDK → Autonomous Robot
```

### Raspberry Pi Integration  
```
Raspberry Pi → Edge Computing + Z1 SDK → Distributed Control
```

## Critical System Dependencies

### ROS Dependencies (Simulation)
```bash
ros-noetic-desktop-full       # Core ROS + Gazebo
ros-noetic-controller-manager # Joint controllers
ros-noetic-gazebo-ros-control # Simulation interface
```

### SDK Dependencies (Real Robot)
```bash
libZ1_SDK_x86_64.so          # Hardware communication library
unitree_arm_interface        # Python SDK wrapper
```

### Python Dependencies (Both)
```bash
numpy scipy matplotlib       # Scientific computing
flask fastapi               # Web interfaces  
opencv-python               # Computer vision (Jetson)
```

## Network Architecture

### Local Development
```
PC (ROS + Gazebo) → localhost → Control Scripts
```

### Remote Control
```
Client → HTTP API (Port 8000) → Z1 Web Service → Robot
```

### Distributed System
```
PC (Planning) ↔ Jetson (AI) ↔ Pi (Sensors) → Z1 Robot
```

## Safety Systems

### Simulation Safety
- Virtual environment prevents hardware damage
- Emergency stop via ESC key
- Joint limit enforcement in software

### Real Robot Safety  
- Hardware emergency stop required
- Workspace limit monitoring
- Force/torque limit checking
- Collision detection algorithms

## Performance Optimization

### For Simulation
- Use headless Gazebo (gui:=false) for better performance
- Reduce physics update rate if needed
- Clear Gazebo cache regularly

### For Real Robot
- Use pure SDK for maximum performance
- Implement real-time control loops
- Monitor system resources
- Use appropriate control frequencies

## Deployment Strategies

### Development Deployment
1. Install full ROS + Gazebo environment
2. Use simulation for safe development
3. Test algorithms without hardware risk

### Production Deployment  
1. Install minimal SDK dependencies only
2. Use pure SDK scripts for real-time control
3. Deploy on embedded systems if needed

### Research Deployment
1. Hybrid approach - develop in simulation
2. Validate on real hardware
3. Compare simulation vs reality performance

## Troubleshooting Knowledge

### Common Issues
- **Gazebo won't start**: Clear cache, check graphics drivers
- **Controllers not loading**: Verify ROS workspace sourcing
- **SDK not found**: Check library paths and permissions
- **Real robot not responding**: Verify network connection and SDK setup

### Debug Commands
```bash
./system_check.sh           # Comprehensive system verification
rostopic list               # Check ROS communication
rostopic echo /joint_states # Monitor robot state
```

## Future Architecture Considerations

### Scalability
- Multi-robot coordination protocols
- Distributed computing architectures
- Cloud integration possibilities

### Performance
- Real-time operating system integration
- Hardware acceleration opportunities
- Network latency optimization

### Safety
- Formal verification methods
- Redundant safety systems
- Human-robot interaction protocols

This architecture provides maximum flexibility while maintaining clear separation between development and production environments.