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
./quick_start.sh keyboard    # Manual control
./quick_start.sh web         # Browser GUI
./quick_start.sh visual      # Visual programming  
./quick_start.sh bartender   # Cocktail demo
./quick_start.sh demo        # Pick & place
```

## Controls
- **WASD** = Move arm
- **ZE** = Elbow
- **Space/X** = Gripper
- **ESC** = Stop

## Problems?
```bash
./fix_gazebo.sh && ./quick_start.sh
```

## Web Interfaces
- **Web Control:** http://localhost:8080
- **Visual Programming:** http://localhost:8081

That's it! 🎯