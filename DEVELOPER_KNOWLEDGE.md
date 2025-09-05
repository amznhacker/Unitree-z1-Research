# Developer Knowledge Base

## Critical Implementation Details

### Script Execution Logic in quick_start.sh

The launcher uses a **case statement mapping** to route commands to appropriate scripts:

```bash
case $CONTROL_METHOD in
    keyboard|k)
        rosrun z1_tools z1_simple_control.py    # ROS-based simulation control
        ;;
    pure|p)  
        python3 src/z1_tools/scripts/z1_pure_sdk_control.py  # Direct SDK execution
        ;;
esac
```

**Key Insight**: Not all scripts use ROS! The execution method depends on the script's purpose:
- **Simulation scripts**: Use `rosrun` (require ROS environment)
- **Production scripts**: Use `python3` directly (pure SDK)

### ROS vs SDK Script Classification

| Script Type | Execution Method | Use Case | Performance |
|-------------|------------------|----------|-------------|
| **ROS Scripts** | `rosrun z1_tools script.py` | Simulation, Development | 50-100Hz, 10-50ms latency |
| **SDK Scripts** | `python3 path/to/script.py` | Real Robot, Production | 500+Hz, <1ms latency |

### Workspace Setup Critical Details

```bash
# Why we need both locations:
~/catkin_ws/src/z1_tools/scripts/    # ROS workspace (for rosrun)
./src/z1_tools/scripts/              # Git repo (for direct python3)
```

**Critical**: ROS scripts must be in catkin workspace, but pure SDK scripts can run from anywhere.

## SDK Integration Patterns

### Pattern 1: ROS Bridge (Development)
```python
import rospy
import unitree_arm_interface

# Bridge ROS topics to SDK calls
def joint_callback(msg):
    arm.MoveJ(msg.data, gripper_pos, speed)

rospy.Subscriber("/joint_command", Float64MultiArray, joint_callback)
```

### Pattern 2: Pure SDK (Production)
```python
import unitree_arm_interface

# Direct SDK control
arm = unitree_arm_interface.ArmInterface(hasGripper=True)
arm.loopOn()
arm.MoveJ(target_posture, gripper_pos, speed)
```

### Pattern 3: Hybrid Control
```python
# Use SDK for kinematics, ROS for coordination
arm_model = arm._ctrlComp.armModel
T = arm_model.forwardKinematics(q, 6)  # SDK kinematics
pub.publish(pose_msg)                   # ROS communication
```

## Hardware Communication Layers

```
Application Layer:     Python Scripts (z1_*.py)
    ↓
SDK Layer:            unitree_arm_interface (C++/Python)
    ↓  
Communication:       USB/Ethernet → Z1 Hardware
    ↓
Robot Layer:         Z1 Arm Controllers & Motors
```

## Control Modes Available

### High-Level SDK Commands
```python
arm.MoveJ(posture, gripper, speed)     # Joint space movement
arm.MoveL(posture, gripper, speed)     # Linear movement  
arm.MoveC(mid, end, gripper, speed)    # Circular movement
```

### Low-Level SDK Commands
```python
arm.setFsmLowcmd()                     # Enable low-level mode
arm.q = target_joints                  # Set joint positions
arm.qd = target_velocities             # Set joint velocities  
arm.tau = target_torques               # Set joint torques
arm.sendRecv()                         # Execute command
```

### ROS Control (Simulation Only)
```python
pub = rospy.Publisher("/z1_gazebo/Joint01_controller/command", Float64)
pub.publish(Float64(joint_angle))      # Send to simulation
```

## Network Protocols

### SDK Communication
- **Protocol**: Custom UDP/TCP
- **IP**: 192.168.123.110 (default robot IP)
- **Frequency**: Up to 500Hz
- **Latency**: <1ms local network

### ROS Communication  
- **Protocol**: TCPROS/UDPROS
- **Transport**: localhost (simulation)
- **Frequency**: 10-100Hz typical
- **Latency**: 10-50ms

### Web API Communication
- **Protocol**: HTTP/WebSocket
- **Port**: 8000 (FastAPI default)
- **Format**: JSON REST API
- **Use**: Remote control, monitoring

## Error Handling Patterns

### SDK Error Handling
```python
try:
    success = arm.MoveJ(posture, gripper, speed)
    if not success:
        print("Movement failed - check workspace limits")
except Exception as e:
    print(f"SDK Error: {e}")
    arm.backToStart()  # Safe recovery
```

### ROS Error Handling
```python
try:
    rospy.init_node('z1_control')
    # ROS operations
except rospy.ROSException as e:
    rospy.logerr(f"ROS Error: {e}")
except KeyboardInterrupt:
    rospy.loginfo("Shutting down gracefully")
```

## Performance Optimization Techniques

### For Real-Time Control
```python
# Use appropriate control frequency
rate = rospy.Rate(100)  # 100Hz for ROS
time.sleep(0.002)       # 500Hz for SDK

# Minimize computation in control loop
# Pre-calculate trajectories
# Use efficient data structures
```

### For Simulation Performance
```bash
# Launch headless Gazebo
roslaunch unitree_gazebo z1.launch gui:=false

# Clear Gazebo cache regularly  
rm -rf ~/.gazebo/log/*
rm -rf /tmp/gazebo*
```

## Safety Implementation

### Software Safety Limits
```python
# Joint limits (radians)
JOINT_LIMITS = {
    'Joint01': (-1.2, 1.2),   # Base rotation
    'Joint02': (-1.0, 1.0),   # Shoulder pitch  
    'Joint03': (0.0, 2.4),    # Elbow
    'Joint04': (-1.2, 1.2),   # Forearm roll
    'Joint05': (-1.0, 1.0),   # Wrist pitch
    'Joint06': (-1.2, 1.2),   # Wrist roll
}

def check_joint_limits(q):
    for i, (min_val, max_val) in enumerate(JOINT_LIMITS.values()):
        if not (min_val <= q[i] <= max_val):
            return False
    return True
```

### Emergency Stop Implementation
```python
def emergency_stop():
    # ROS version
    for joint in joint_publishers:
        joint.publish(Float64(0.0))
    
    # SDK version  
    arm.setFsm(unitree_arm_interface.ArmFSMState.PASSIVE)
```

## Debugging Techniques

### ROS Debugging
```bash
# Check ROS communication
rostopic list | grep z1_gazebo
rostopic echo /z1_gazebo/joint_states
rostopic hz /z1_gazebo/joint_states

# Monitor system
rqt_graph                    # Visualize ROS nodes
rqt_plot /joint_states/position[0]  # Plot joint data
```

### SDK Debugging
```python
# Monitor SDK state
print(f"Current joints: {arm.lowstate.getQ()}")
print(f"Gripper state: {arm.lowstate.getGripperQ()}")

# Check SDK connection
if arm.lowstate is None:
    print("SDK not connected to robot")
```

## Integration Patterns

### Jetson AI Integration
```python
# Jetson-specific imports
import jetson.inference
import jetson.utils

# Combine AI with robot control
detections = net.Detect(img)
for detection in detections:
    target_pos = calculate_3d_position(detection)
    arm.MoveL(target_pos, gripper_pos, speed)
```

### Raspberry Pi Integration
```python
# GPIO sensor integration
import RPi.GPIO as GPIO

# Read sensors and control robot
force_reading = read_force_sensor()
if force_reading > threshold:
    arm.setFsm(ArmFSMState.PASSIVE)  # Safety stop
```

## Common Pitfalls & Solutions

### Pitfall 1: Mixed ROS/SDK Execution
**Problem**: Using `rosrun` for pure SDK scripts
**Solution**: Check script dependencies, use appropriate execution method

### Pitfall 2: Workspace Sourcing
**Problem**: ROS scripts fail with "package not found"
**Solution**: Always source workspace: `source ~/catkin_ws/devel/setup.bash`

### Pitfall 3: SDK Library Path
**Problem**: "libZ1_SDK_x86_64.so not found"
**Solution**: Check library is in correct path, verify permissions

### Pitfall 4: Network Configuration
**Problem**: Cannot connect to real robot
**Solution**: Verify IP address (192.168.123.110), check network settings

## Testing Strategies

### Unit Testing
```python
def test_joint_limits():
    assert check_joint_limits([0, 0, 1, 0, 0, 0]) == True
    assert check_joint_limits([2, 0, 0, 0, 0, 0]) == False

def test_kinematics():
    T = arm_model.forwardKinematics(np.zeros(6), 6)
    assert T.shape == (4, 4)
```

### Integration Testing
```bash
# Test full system
./system_check.sh

# Test individual components
rostopic echo /z1_gazebo/joint_states -n 1
python3 -c "import unitree_arm_interface; print('SDK OK')"
```

This knowledge base captures the critical implementation details that every developer working on the Z1 system should understand.