# Unitree Z1 Robotic Arm

Complete ROS workspace for Unitree Z1 simulation and real robot control on Ubuntu 20.04.6 LTS.

## 🚀 Quick Start

```bash
git clone git@github.com:amznhacker/Unitree-z1-Research.git
cd Unitree-z1-Research
chmod +x setup_and_run.sh
./setup_and_run.sh --install-ros
```

## 🎮 Usage

### One Command (Recommended)
```bash
./quick_start.sh keyboard    # Keyboard control
./quick_start.sh demo        # Pick & place demo
./quick_start.sh bartender   # Cocktail demo
./quick_start.sh draw        # Drawing demo
./quick_start.sh real        # Real robot
```

### Two Terminals (Advanced)
```bash
# Terminal 1: Start simulation
roslaunch unitree_gazebo z1.launch

# Terminal 2: Run any script
rosrun z1_tools z1_bartender.py
rosrun z1_tools z1_chess_player.py
rosrun z1_tools z1_simple_control.py
```

## 🎯 Controls

- **WASD** = Base/Shoulder
- **ZE** = Elbow  
- **RF** = Forearm
- **TG** = Wrist pitch
- **YH** = Wrist roll
- **Space/X** = Gripper open/close
- **ESC** = Stop

## 🔧 Troubleshooting

```bash
# Gazebo issues
pkill -f gazebo && ./quick_start.sh

# Build issues  
cd ~/catkin_ws && catkin_make clean && catkin_make

# Permission issues
sudo chown -R $USER:$USER ~/catkin_ws
```

## 📁 Available Scripts

**Control:** keyboard, simple_control, xbox_control  
**Demos:** demo_simple, pick_place, drawing  
**Entertainment:** bartender, chess_player, magician, musician, wave  
**Utilities:** emergency_stop, safe_limits, motorcmd_sweep  

See [TWO_TERMINAL_USAGE.md](TWO_TERMINAL_USAGE.md) for complete list.

## ⚠️ Safety

- **Simulation by default** - completely safe
- **Real robot mode** - requires safety training and setup
- **Emergency stop** - ESC key or Ctrl+C

---

**Perfect for learning robotics - safe simulation with real robot capability!**