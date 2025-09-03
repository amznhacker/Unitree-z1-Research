# 🚀 Unitree Z1 - Start Here

## One-Command Setup & Launch

### First Time (Install Everything)
```bash
git clone git@github.com:amznhacker/Unitree-z1-Research.git
cd Unitree-z1-Research
chmod +x setup_and_run.sh
./setup_and_run.sh --install-ros
```

### Daily Use (Quick Start)
```bash
./quick_start.sh keyboard    # Keyboard control
./quick_start.sh xbox        # Xbox controller  
./quick_start.sh demo        # Pick & place demo
./quick_start.sh draw        # Drawing demo
./quick_start.sh real        # Real robot mode
```

## 🎮 Controls

**Keyboard:**
- WASD = Base/Shoulder
- ZE = Elbow
- RF = Forearm roll
- TG = Wrist pitch
- YH = Wrist roll
- Space = Open gripper
- X = Close gripper
- ESC = Stop

## 🔧 Troubleshooting

**Permission errors:**
```bash
sudo chown -R $USER:$USER ~/catkin_ws
```

**Gazebo won't start:**
```bash
pkill -f gazebo && ./quick_start.sh
```

**Build errors:**
```bash
cd ~/catkin_ws && catkin_make clean && catkin_make
```

## ⚠️ Safety Warning

**SIMULATION ONLY** by default - These scripts are for Gazebo simulation.
For real hardware, use `./quick_start.sh real` and follow safety procedures.

---

**Perfect! Two commands and you're running the Z1 simulator.**