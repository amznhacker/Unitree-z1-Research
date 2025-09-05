# 🚀 START HERE - Unitree Z1 Quick Guide

## Step 1: Setup (One Time Only)
```bash
git clone https://github.com/amznhacker/Unitree-z1-Research.git
cd Unitree-z1-Research
chmod +x setup_and_run.sh
./setup_and_run.sh --install-ros
```
*This installs ROS, builds the workspace, and sets up everything automatically.*

## Step 2: Choose Your Experience

### 🎮 **New to Robotics?** Start with demos:
```bash
./quick_start.sh demo        # Automated pick & place
./quick_start.sh bartender   # Impressive cocktail mixing
./quick_start.sh draw        # Drawing shapes
```

### 🕹️ **Want to Control?** Try manual control:
```bash
./quick_start.sh keyboard    # WASD keys control
./quick_start.sh xbox        # Xbox controller
./quick_start.sh web         # Browser interface
```

### 🚀 **Advanced User?** Explore enhanced features:
```bash
./quick_start.sh enhanced    # Full SDK integration
./quick_start.sh api         # Web API server
./leverage_everything.sh     # Complete demonstration
```

### 🎭 **Entertainment?** Fun demos:
```bash
./quick_start.sh chess       # Chess playing
./quick_start.sh magic       # Magic tricks
./quick_start.sh music       # Musical movements
```

## Step 3: See All Options
```bash
./quick_start.sh list        # Show all available scripts
./system_check.sh           # Verify everything works
```

## 🎯 Controls (Keyboard Mode)
- **WASD** = Move arm base and shoulder
- **ZE** = Elbow bend/extend  
- **RF** = Forearm roll
- **TG** = Wrist pitch
- **YH** = Wrist roll
- **Space** = Open gripper
- **X** = Close gripper
- **ESC** = Emergency stop

## 🔧 Problems?
```bash
./fix_gazebo.sh             # Fix Gazebo issues
./system_check.sh           # Check all components
```

## 📚 Learn More
- **README.md** - Complete documentation
- **deployment_guide.md** - Advanced deployment
- **research_implementations.md** - Academic research

---

**That's it! You're ready to explore the world of robotic manipulation!**