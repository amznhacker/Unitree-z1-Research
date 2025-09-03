# Unitree Z1 Launch Scripts

## First Time Setup

Run the complete setup script (only needed once):

```bash
chmod +x setup_and_run.sh
./setup_and_run.sh --install-ros --control-method keyboard
```

### Setup Script Options:
- `--install-ros`: Install ROS Noetic (first time only)
- `--control-method keyboard`: Use keyboard control
- `--control-method xbox`: Use Xbox controller
- `--control-method demo`: Run pick & place demo

## Daily Use

For quick startup after initial setup:

```bash
chmod +x quick_start.sh

# Keyboard control (default)
./quick_start.sh

# Xbox controller
./quick_start.sh xbox

# Pick and place demo
./quick_start.sh demo

# Drawing demo
./quick_start.sh draw

# Chess demo
./quick_start.sh chess

# Other options
./quick_start.sh wave
./quick_start.sh bartender
```

## What the Scripts Do

### setup_and_run.sh (First Time)
1. ✅ Install ROS Noetic (if --install-ros flag used)
2. ✅ Install all required ROS packages
3. ✅ Set up catkin workspace
4. ✅ Copy source files
5. ✅ Build workspace with catkin_make
6. ✅ Fix Gazebo world file paths
7. ✅ Make all scripts executable
8. ✅ Build Z1 SDK
9. ✅ Setup Xbox controller (if needed)
10. ✅ Launch Gazebo with Z1 robot
11. ✅ Start chosen control method

### quick_start.sh (Daily Use)
1. ✅ Source workspace
2. ✅ Kill any existing ROS/Gazebo processes
3. ✅ Launch Gazebo with Z1
4. ✅ Start control interface

## Control Methods

| Method | Command | Description |
|--------|---------|-------------|
| `keyboard` | `./quick_start.sh keyboard` | WASD movement, Space/X gripper |
| `xbox` | `./quick_start.sh xbox` | Xbox gamepad control |
| `demo` | `./quick_start.sh demo` | Automated pick & place |
| `pick` | `./quick_start.sh pick` | 3-cycle pick & place |
| `draw` | `./quick_start.sh draw` | Draw geometric shapes |
| `chess` | `./quick_start.sh chess` | Chess playing demo |
| `bartender` | `./quick_start.sh bartender` | Cocktail mixing demo |
| `wave` | `./quick_start.sh wave` | Simple wave motion |

## Keyboard Controls
- **W/S**: Shoulder pitch up/down
- **A/D**: Base rotate left/right  
- **Q/E**: Elbow bend/extend
- **R/F**: Forearm roll
- **T/G**: Wrist pitch
- **Y/H**: Wrist roll
- **Space**: Open gripper
- **X**: Close gripper
- **ESC**: Emergency stop

## Troubleshooting

### If Gazebo won't start:
```bash
pkill -f gazebo
./quick_start.sh
```

### If workspace build fails:
```bash
cd ~/catkin_ws
catkin_make clean
catkin_make
```

### If scripts aren't executable:
```bash
chmod +x *.sh
```

### Check ROS installation:
```bash
roscore &
sleep 2
rostopic list
killall roscore
```

## Quick Commands

```bash
# Complete first-time setup
./setup_and_run.sh --install-ros

# Daily keyboard control
./quick_start.sh

# Daily Xbox control  
./quick_start.sh xbox

# Run a demo
./quick_start.sh demo
```

## Stop Everything
Press `Ctrl+C` in the terminal or `ESC` in keyboard control mode.