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
./quick_start.sh keyboard    # Manual control
./quick_start.sh xbox        # Xbox controller
./quick_start.sh web         # Browser control
./quick_start.sh visual      # Visual programming
./quick_start.sh demo        # Pick & place demo
./quick_start.sh bartender   # Cocktail mixing
./quick_start.sh draw        # Drawing shapes
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

## 🔧 Problems?

```bash
# If Gazebo won't start or robot won't spawn
./fix_gazebo.sh && ./quick_start.sh

# If build errors
cd ~/catkin_ws && catkin_make clean && catkin_make

# If permission errors
sudo chown -R $USER:$USER ~/catkin_ws
```

## 📁 Available Scripts

**Control:** keyboard, xbox_control, web_gui, visual_programmer  
**Demos:** demo_simple, pick_place, drawing, bartender  
**Entertainment:** chess_player, magician, musician, wave  
**Utilities:** emergency_stop, safe_limits, motorcmd_sweep  

See [TWO_TERMINAL_USAGE.md](TWO_TERMINAL_USAGE.md) for complete list.

## ⚠️ Safety

- **Simulation by default** - completely safe
- **Real robot mode** - requires safety training and setup
- **Emergency stop** - ESC key or Ctrl+C

---

**Perfect for learning robotics - safe simulation with real robot capability!**