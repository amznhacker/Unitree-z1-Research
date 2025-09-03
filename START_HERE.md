# 🚀 Unitree Z1 - Start Here

## One-Command Setup & Launch

### First Time (Install Everything)
```bash
git clone https://github.com/YOUR_USERNAME/unitree_z1_workspace.git
cd unitree_z1_workspace
chmod +x setup_and_run.sh
./setup_and_run.sh --install-ros
```

### Daily Use (Quick Start)
```bash
./quick_start.sh keyboard    # Keyboard control
./quick_start.sh xbox        # Xbox controller
./quick_start.sh demo        # Pick & place demo
./quick_start.sh draw        # Drawing demo
```

## 🎮 Controls

**Keyboard:**
- WASD = Base/Shoulder
- QE = Elbow
- RF = Forearm roll
- TG = Wrist pitch
- YH = Wrist roll
- Space = Open gripper
- X = Close gripper
- ESC = Stop

## 🔧 Troubleshooting

**Gazebo won't start:**
```bash
pkill -f gazebo && ./quick_start.sh
```

**Build errors:**
```bash
cd ~/catkin_ws && catkin_make clean && catkin_make
```

## ⚠️ Safety Warning

**SIMULATION ONLY** - These scripts are for Gazebo simulation.
For real hardware, see [REAL_HARDWARE_GUIDE.md](REAL_HARDWARE_GUIDE.md)

---

That's it! Two commands and you're running the Z1 simulator.