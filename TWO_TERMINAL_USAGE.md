# Two Terminal Usage Guide

## 🚀 Quick Method (One Terminal)
```bash
./quick_start.sh keyboard    # or demo, draw, bartender, real
```

## 🎯 Advanced Method (Two Terminals)

### Terminal 1: Start Gazebo
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch unitree_gazebo z1.launch
```
*Keep this terminal open - Gazebo stays running*

### Terminal 2: Run Any Script
```bash
source ~/catkin_ws/devel/setup.bash

# Control Scripts
rosrun z1_tools z1_simple_control.py      # Keyboard control
rosrun z1_tools z1_xbox_control.py        # Xbox controller

# Demo Scripts  
rosrun z1_tools z1_demo_simple.py         # Basic pick & place
rosrun z1_tools z1_pick_place.py          # Advanced pick & place
rosrun z1_tools z1_drawing.py             # Draw shapes

# Entertainment Scripts
rosrun z1_tools z1_bartender.py           # Cocktail mixing
rosrun z1_tools z1_chess_player.py        # Chess moves
rosrun z1_tools z1_magician.py            # Magic tricks
rosrun z1_tools z1_musician.py            # Musical movements
rosrun z1_tools z1_wave.py                # Wave motion
rosrun z1_tools z1_cigar_roller.py        # Cigar rolling
rosrun z1_tools z1_cumbia.py              # Dance moves

# Utility Scripts
rosrun z1_tools z1_emergency_stop.py      # Emergency stop
rosrun z1_tools z1_safe_limits.py         # Test joint limits
rosrun z1_tools z1_motorcmd_sweep.py      # Joint sweep test
```

## 📋 Available Scripts in z1_tools/scripts/

### Control Scripts:
- `z1_simple_control.py` - Basic keyboard control (WASD, ZE, etc.)
- `z1_xbox_control.py` - Xbox controller support
- `z1_web_gui.py` - Browser-based control interface

### Demo Scripts:
- `z1_demo_simple.py` - Basic pick and place demonstration
- `z1_pick_place.py` - Advanced pick and place with multiple cycles
- `z1_drawing.py` - Draw geometric shapes (circles, squares, figure-8)

### Entertainment Scripts:
- `z1_bartender.py` - Cocktail mixing choreography
- `z1_chess_player.py` - Chess piece movement simulation
- `z1_magician.py` - Magic trick performances
- `z1_musician.py` - Musical conducting motions
- `z1_wave.py` - Simple wave and greeting motions
- `z1_cigar_roller.py` - Artisan cigar rolling simulation
- `z1_cumbia.py` - Dance movement patterns

### Utility Scripts:
- `z1_emergency_stop.py` - Immediate stop all joints
- `z1_safe_limits.py` - Test and display joint limits
- `z1_motorcmd_sweep.py` - Sweep individual joints for testing
- `z1_capabilities_demo.py` - Show all robot capabilities
- `z1_visual_programmer.py` - Visual programming interface

### Real Robot Scripts:
- `z1_real_robot_bridge.py` - Bridge simulation to real robot

## 🎮 Control Instructions

### Keyboard Control (z1_simple_control.py):
- **WASD** = Base rotation & shoulder pitch
- **ZE** = Elbow bend/extend
- **RF** = Forearm roll
- **TG** = Wrist pitch
- **YH** = Wrist roll
- **Space** = Open gripper
- **X** = Close gripper
- **ESC** = Emergency stop
- **Q** = Quit

### Xbox Controller (z1_xbox_control.py):
- **Left Stick** = Base rotation & shoulder pitch
- **Right Stick** = Elbow & forearm roll
- **D-Pad** = Wrist pitch & roll
- **RT/LT** = Gripper open/close
- **Back Button** = Emergency stop

## 🔧 Troubleshooting

### Script won't start:
```bash
# Make sure Gazebo is running first
rostopic list | grep z1_gazebo

# Check if controllers are loaded
rostopic echo /z1_gazebo/joint_states
```

### Robot not moving:
```bash
# Check message types match
rostopic info /z1_gazebo/Joint01_controller/command

# Verify Gazebo is unpaused
rosservice call /gazebo/unpause_physics
```

### Permission errors:
```bash
chmod +x ~/catkin_ws/src/z1_tools/scripts/*.py
```

## 💡 Pro Tips

1. **Keep Gazebo running** in Terminal 1, switch scripts in Terminal 2
2. **Use Ctrl+C** to stop any script safely
3. **Source workspace** in each new terminal
4. **Check joint_states** topic to verify robot is responding
5. **Start with simple scripts** before trying complex demos

## 🚀 Quick Examples

```bash
# Terminal 1
roslaunch unitree_gazebo z1.launch

# Terminal 2 - Try different scripts
rosrun z1_tools z1_wave.py           # Wave hello
rosrun z1_tools z1_drawing.py        # Draw shapes  
rosrun z1_tools z1_bartender.py      # Mix cocktails
rosrun z1_tools z1_simple_control.py # Manual control
```