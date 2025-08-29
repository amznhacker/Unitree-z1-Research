# Z1 Robotic Arm Control Scripts

This package contains various control scripts for the Unitree Z1 robotic arm in ROS/Gazebo simulation.

## Prerequisites

1. Launch Z1 in Gazebo:
```bash
roslaunch unitree_gazebo z1.launch
```

2. Make scripts executable:
```bash
chmod +x scripts/*.py
```

## Available Scripts

### 1. Xbox Controller (`z1_xbox_control.py`)
Real-time control with Xbox gamepad - most intuitive control method!

**Setup:**
```bash
# Install ROS joy package
sudo apt-get install ros-noetic-joy

# Run setup script (Linux only)
chmod +x scripts/setup_xbox_controller.sh
./scripts/setup_xbox_controller.sh
```

**Usage:**
```bash
# Easy launch (includes joy node)
roslaunch z1_tools z1_xbox_control.launch

# Or manual launch
rosrun joy joy_node &
rosrun z1_tools z1_xbox_control.py
```

**Xbox Controls:**
- **Left Stick**: Base rotation (X) + Shoulder pitch (Y)
- **Right Stick**: Elbow (X) + Wrist pitch (Y)
- **LT/RT Triggers**: Forearm roll left/right
- **LB/RB Bumpers**: Wrist roll left/right
- **A Button**: Close gripper
- **B Button**: Open gripper
- **X Button**: Toggle precision mode
- **Y Button**: Emergency stop
- **Start**: Increase speed
- **Back**: Decrease speed

### 2. Keyboard Control (`z1_keyboard_control.py`)
Real-time keyboard control of the Z1 arm with safety limits.

**Usage:**
```bash
rosrun z1_tools z1_keyboard_control.py
```

**Controls:**
- `W/S`: Shoulder pitch up/down
- `A/D`: Base rotate left/right  
- `Q/E`: Elbow bend/extend
- `R/F`: Forearm roll CCW/CW
- `T/G`: Wrist pitch up/down
- `Y/H`: Wrist roll CCW/CW
- `Space`: Open gripper
- `X`: Close gripper
- `1-6`: Select joint for fine control
- `Arrow keys`: Fine control selected joint
- `ESC`: Emergency stop
- `Q`: Quit

### 3. Pick and Place Demo (`z1_pick_place.py`)
Automated pick and place sequence with predefined safe waypoints.

**Usage:**
```bash
# Single cycle
rosrun z1_tools z1_pick_place.py

# Multiple cycles
rosrun z1_tools z1_pick_place.py --cycles 3

# Custom parameters
rosrun z1_tools z1_pick_place.py --mode 10 --kp 30.0 --kd 2.0
```

### 4. Safe Limits Tester (`z1_safe_limits.py`)
Test all joints within conservative safe operating limits.

**Usage:**
```bash
# Test all joints individually
rosrun z1_tools z1_safe_limits.py --test individual

# Sine sweep all joints
rosrun z1_tools z1_safe_limits.py --test sweep --duration 30

# Test specific joint
rosrun z1_tools z1_safe_limits.py --joint Joint02

# Both tests
rosrun z1_tools z1_safe_limits.py --test both
```

### 5. Drawing Demo (`z1_drawing.py`)
Make the arm draw geometric shapes in 3D space.

**Usage:**
```bash
# Draw all shapes
rosrun z1_tools z1_drawing.py

# Draw specific shape
rosrun z1_tools z1_drawing.py --shape circle
rosrun z1_tools z1_drawing.py --shape figure8
rosrun z1_tools z1_drawing.py --shape square

# Custom size and speed
rosrun z1_tools z1_drawing.py --shape circle --size 0.15 --speed 1.5
```

### 6. Emergency Stop (`z1_emergency_stop.py`)
Immediate safety shutdown - stops all motion and returns to neutral.

**Usage:**
```bash
# Immediate stop
rosrun z1_tools z1_emergency_stop.py

# Monitor mode (press Ctrl+C to activate)
rosrun z1_tools z1_emergency_stop.py --monitor
```

### 7. Entertainment Scripts

**Chess Player (`z1_chess_player.py`):**
```bash
# Full chess game simulation
rosrun z1_tools z1_chess_player.py

# Single move
rosrun z1_tools z1_chess_player.py --move e2 e4

# Specific gesture
rosrun z1_tools z1_chess_player.py --gesture victory
```

**Cigar Roller (`z1_cigar_roller.py`):**
```bash
# Full artisan sequence
rosrun z1_tools z1_cigar_roller.py

# Specific step
rosrun z1_tools z1_cigar_roller.py --step roll --rolls 8
```

**Bartender (`z1_bartender.py`):**
```bash
# Full cocktail show
rosrun z1_tools z1_bartender.py

# Specific cocktail
rosrun z1_tools z1_bartender.py --cocktail martini
```

**Musician (`z1_musician.py`):**
```bash
# Full concert
rosrun z1_tools z1_musician.py

# Specific instrument
rosrun z1_tools z1_musician.py --instrument violin
rosrun z1_tools z1_musician.py --instrument conductor --tempo 140
```

**Magician (`z1_magician.py`):**
```bash
# Full magic show
rosrun z1_tools z1_magician.py

# Specific trick
rosrun z1_tools z1_magician.py --trick rabbit
```

### 8. Original Scripts

**Wave Motion (`z1_wave.py`):**
```bash
rosrun z1_tools z1_wave.py
```

**Cumbia Dance (`z1_cumbia.py`):**
```bash
rosrun z1_tools z1_cumbia.py --bpm 120 --dur 30
```

**Motor Sweep (`z1_motorcmd_sweep.py`):**
```bash
rosrun z1_tools z1_motorcmd_sweep.py --joint Joint02 --amp 0.5 --freq 0.3
```

## Safety Features

All scripts include:
- **Conservative joint limits** to prevent damage
- **Smooth interpolation** to avoid sudden movements  
- **Emergency stop capability** (ESC key or Ctrl+C)
- **Gradual startup/shutdown** sequences
- **Position clamping** within safe ranges

## Joint Limits (Conservative)

| Joint | Range | Description |
|-------|-------|-------------|
| Joint01 | ±69° | Base yaw rotation |
| Joint02 | ±57° | Shoulder pitch |
| Joint03 | 0° to 137° | Elbow bend |
| Joint04 | ±69° | Forearm roll |
| Joint05 | ±57° | Wrist pitch |
| Joint06 | ±69° | Wrist roll |
| Gripper | 0-60% | Gripper opening |

## Control Parameters

- **Mode**: Usually 10 (position control)
- **Kp**: Proportional gain (default: 35.0)
- **Kd**: Derivative gain (default: 1.5)
- **Rate**: 50 Hz control loop

## Troubleshooting

1. **No motion**: Try `--mode 5` instead of default mode 10
2. **Jerky movement**: Reduce Kp gain or increase Kd
3. **Arm stuck**: Run emergency stop script
4. **Gazebo not responding**: Restart simulation

## Development Notes

- All scripts use `unitree_legged_msgs/MotorCmd` messages
- Topics follow pattern: `/z1_gazebo/{Joint}_controller/command`
- Scripts are designed for simulation - adapt for real hardware
- Always test in simulation before real robot use

## Safety Warning

⚠️ **IMPORTANT**: These scripts are designed for simulation. When using with real hardware:
- Reduce gains and speeds significantly
- Test individual joints first
- Have emergency stop readily available
- Ensure clear workspace around robot
- Follow all manufacturer safety guidelines