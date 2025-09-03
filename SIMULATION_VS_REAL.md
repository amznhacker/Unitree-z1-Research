# Simulation vs Real Hardware Guide

## 🎮 SIMULATION (Current Setup)

### What is Simulation?
- **Gazebo**: 3D physics simulator that mimics real robot behavior
- **No Physical Robot**: Everything runs on computer
- **Safe Testing**: No risk of damage or injury
- **Perfect for Learning**: Experiment freely without consequences

### Current Scripts are SIMULATION ONLY:
```bash
./quick_start.sh keyboard    # ✅ SIMULATION - Safe to use
./quick_start.sh xbox        # ✅ SIMULATION - Safe to use  
./quick_start.sh demo        # ✅ SIMULATION - Safe to use
```

### Simulation Characteristics:
- **High gains**: Kp=35.0, Kd=1.5 (aggressive control)
- **Fast movements**: No velocity limits
- **No safety checks**: Assumes perfect environment
- **Instant response**: No communication delays
- **No physical constraints**: Can push through limits

---

## 🤖 REAL HARDWARE (Requires Special Setup)

### ⚠️ CRITICAL DIFFERENCES

| Aspect | Simulation | Real Hardware |
|--------|------------|---------------|
| **Safety** | No risk | HIGH RISK - Can cause injury/damage |
| **Parameters** | Kp=35, fast moves | Kp=8, slow moves |
| **Limits** | Soft limits | HARD LIMITS - can break robot |
| **Emergency Stop** | Ctrl+C | Physical button + software |
| **Environment** | Perfect | Real physics, obstacles |
| **Consequences** | None | Expensive repairs, injury |

### Real Hardware Requirements:
1. **Physical Z1 Robot** (not included in this workspace)
2. **Real Hardware Interface** (`unitree_ros_to_real` package)
3. **Safety Training** and certification
4. **Controlled Environment** with safety barriers
5. **Emergency Stop Systems** (hardware + software)

---

## 🚨 SAFETY REQUIREMENTS FOR REAL HARDWARE

### Before ANY Real Robot Operation:

#### 1. Physical Safety Setup
- [ ] **Emergency stop button** within arm's reach
- [ ] **Safety barriers** around robot workspace  
- [ ] **Clear workspace** - no people/objects within 3 meters
- [ ] **Protective equipment** (safety glasses, etc.)
- [ ] **Trained operator** present at all times

#### 2. Software Safety Setup
- [ ] **Conservative parameters**: Kp ≤ 10, max velocity ≤ 0.5 rad/s
- [ ] **Joint limit checking** enabled and tested
- [ ] **Velocity limiting** active
- [ ] **Position monitoring** with automatic stops
- [ ] **Communication timeout** detection

#### 3. Emergency Stop Systems
- [ ] **Hardware E-stop** button (cuts power immediately)
- [ ] **Software E-stop** (Ctrl+C, ESC key)
- [ ] **Watchdog timer** (stops if no commands received)
- [ ] **Limit switch monitoring** 
- [ ] **Force/torque monitoring** (if available)

---

## 🔧 CONVERTING SIMULATION TO REAL HARDWARE

### Step 1: Install Real Hardware Interface
```bash
# Install unitree_ros_to_real package
cd ~/catkin_ws/src
git clone https://github.com/unitreerobotics/unitree_ros_to_real.git
catkin_make
```

### Step 2: Use Safety Wrapper Script
```bash
# NEVER use simulation scripts directly on real robot
# Use the safety wrapper instead:
python3 REAL_HARDWARE_GUIDE.md  # This is actually a Python script
```

### Step 3: Parameter Conversion

| Simulation Parameter | Real Hardware Parameter |
|---------------------|------------------------|
| `Kp: 35.0` | `Kp: 8.0` (conservative) |
| `Kd: 1.5` | `Kd: 1.0` (conservative) |
| `max_velocity: unlimited` | `max_velocity: 0.3 rad/s` |
| `joint_limits: full range` | `joint_limits: 80% of range` |
| `mode: 10` | `mode: 10` (same) |

### Step 4: Topic Name Changes
```bash
# Simulation topics:
/z1_gazebo/Joint01_controller/command

# Real hardware topics:  
/z1_real/Joint01_controller/command
```

---

## 🛡️ MANDATORY SAFETY CHECKLIST

### Before Starting Real Robot:

#### Physical Environment ✅
- [ ] Emergency stop button tested and accessible
- [ ] Workspace clear (3+ meter radius)
- [ ] No people in danger zone
- [ ] Safety barriers in place
- [ ] Protective equipment worn
- [ ] Fire extinguisher nearby (for electrical fires)

#### Software Safety ✅  
- [ ] Conservative parameters loaded (Kp ≤ 10)
- [ ] Joint limits set to 80% of maximum range
- [ ] Velocity limits active (≤ 0.5 rad/s)
- [ ] Emergency stop handlers installed
- [ ] Communication timeout detection active
- [ ] Position monitoring enabled

#### Operational Safety ✅
- [ ] Trained operator present
- [ ] Communication with robot verified
- [ ] All safety systems tested
- [ ] Emergency procedures reviewed
- [ ] Contact information for support available

---

## 🚫 WHAT NOT TO DO WITH REAL HARDWARE

### NEVER:
- ❌ Use simulation scripts directly on real robot
- ❌ Use high gains (Kp > 15) without extensive testing
- ❌ Operate without emergency stop systems
- ❌ Leave robot unattended during operation
- ❌ Exceed conservative joint limits
- ❌ Ignore safety warnings or bypass safety systems
- ❌ Operate in uncontrolled environments

### ALWAYS:
- ✅ Start with minimal movements
- ✅ Test each joint individually first
- ✅ Use conservative parameters
- ✅ Have emergency stop ready
- ✅ Monitor robot behavior continuously
- ✅ Follow manufacturer safety guidelines

---

## 📋 REAL HARDWARE STARTUP PROCEDURE

### 1. Pre-Operation Checks (15 minutes)
```bash
# Run safety checklist
python3 real_hardware_safety_check.py

# Verify robot communication
rostopic list | grep z1_real

# Test emergency stop
# (Physical button + software Ctrl+C)
```

### 2. Conservative Startup (10 minutes)
```bash
# Start with minimal movement
python3 z1_real_hardware_safe.py --test-individual-joints

# Gradually increase range if successful
python3 z1_real_hardware_safe.py --test-small-movements
```

### 3. Normal Operation
```bash
# Only after successful testing
python3 z1_real_hardware_safe.py --normal-operation
```

---

## 🆘 EMERGENCY PROCEDURES

### If Robot Moves Unexpectedly:
1. **IMMEDIATELY** press hardware emergency stop
2. **IMMEDIATELY** press Ctrl+C in terminal
3. **DO NOT** try to physically stop the robot
4. **WAIT** for robot to completely stop before approaching
5. **INVESTIGATE** cause before restarting

### If Software Freezes:
1. Press hardware emergency stop
2. Kill all ROS processes: `pkill -f ros`
3. Power cycle the robot
4. Restart with conservative parameters

### If Communication Lost:
1. Robot should stop automatically (watchdog timer)
2. If not, use hardware emergency stop
3. Check network connections
4. Restart communication systems

---

## 📞 SUPPORT CONTACTS

- **Unitree Support**: [support.unitree.com](https://support.unitree.com)
- **Emergency**: Your local emergency services
- **Technical**: Your robotics supervisor/instructor

---

## ⚖️ LIABILITY WARNING

**This workspace is provided for educational purposes. Users are responsible for:**
- Following all safety procedures
- Proper training before real hardware use  
- Compliance with local safety regulations
- Any damage or injury resulting from misuse

**Real robot operation requires proper training, safety equipment, and controlled environments.**