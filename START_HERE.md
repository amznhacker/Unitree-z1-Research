# 🚀 Unitree Z1 - Start Here

## Setup (First Time Only)
```bash
git clone git@github.com:amznhacker/Unitree-z1-Research.git
cd Unitree-z1-Research
chmod +x setup_and_run.sh
./setup_and_run.sh --install-ros
```

## Daily Use
```bash
./quick_start.sh keyboard    # Keyboard control
./quick_start.sh web         # Web browser GUI
./quick_start.sh visual      # Visual programming
./quick_start.sh demo        # Pick & place demo
```

## Controls
- **WASD** = Move arm
- **ZE** = Elbow
- **Space/X** = Gripper
- **ESC** = Stop

## Problems?
```bash
pkill -f gazebo && ./quick_start.sh
```

That's it! 🎯