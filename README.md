# Unitree Z1 Robotic Arm ROS Workspace

Complete ROS workspace for Unitree Z1 robotic arm simulation and control on Ubuntu 20.04.6 LTS.

## 🚀 Quick Start

### First Time Setup (One Command)
```bash
git clone https://github.com/YOUR_USERNAME/unitree_z1_workspace.git
cd unitree_z1_workspace
chmod +x setup_and_run.sh
./setup_and_run.sh --install-ros --control-method keyboard
```

### Daily Use (One Command)
```bash
./quick_start.sh keyboard    # or xbox, demo, draw, chess
```

## 📦 What's Included

- **Unitree ROS Packages** - Complete robot descriptions and controllers
- **Z1 SDK** - C++ SDK with Python bindings
- **Z1 Tools** - 15+ control scripts and entertainment demos
- **Automated Setup** - One-command installation and daily startup
- **Multiple Control Methods** - Keyboard, Xbox controller, automated demos

## 🎮 Control Methods

| Method | Command | Description |
|--------|---------|-------------|
| Keyboard | `./quick_start.sh keyboard` | WASD movement, intuitive controls |
| Xbox Controller | `./quick_start.sh xbox` | Gamepad control with triggers/bumpers |
| Pick & Place | `./quick_start.sh demo` | Automated object manipulation |
| Drawing | `./quick_start.sh draw` | Draw circles, squares, figure-8 |
| Chess Player | `./quick_start.sh chess` | Chess piece movement simulation |
| Bartender | `./quick_start.sh bartender` | Cocktail mixing demonstration |
| Musician | `./quick_start.sh wave` | Rhythmic arm movements |

## 🛠 Requirements

- Ubuntu 20.04.6 LTS
- 4GB+ RAM, 20GB+ disk space
- Internet connection for initial setup

## 📋 Manual Installation

If you prefer step-by-step setup, see [SETUP_UBUNTU_20.04.md](SETUP_UBUNTU_20.04.md)

## 🎯 Keyboard Controls

- **W/S**: Shoulder pitch up/down
- **A/D**: Base rotate left/right  
- **Q/E**: Elbow bend/extend
- **R/F**: Forearm roll
- **T/G**: Wrist pitch
- **Y/H**: Wrist roll
- **Space**: Open gripper
- **X**: Close gripper
- **ESC**: Emergency stop

## 🤖 Supported Robots

- **Z1** - 6-DOF robotic arm (main focus)
- **Quadrupeds**: A1, B1, B2, Go1, Go2, Aliengo, Laikago
- **Humanoids**: G1, H1, H1_2
- **Hands**: Dexterous hand descriptions

## 🔧 Troubleshooting

### Gazebo won't start:
```bash
pkill -f gazebo && ./quick_start.sh
```

### Build errors:
```bash
cd ~/catkin_ws && catkin_make clean && catkin_make
```

### Permission issues:
```bash
chmod +x *.sh
```

## 📁 Project Structure

```
unitree_ws/
├── src/
│   ├── unitree_ros/          # Robot descriptions & controllers
│   ├── z1_sdk/               # C++ SDK with Python bindings
│   ├── z1_tools/             # Control scripts & demos
│   └── unitree_ros_to_real/  # Real hardware interface
├── setup_and_run.sh          # Complete setup script
├── quick_start.sh             # Daily use script
└── README_SCRIPTS.md          # Detailed script usage
```

## 🎪 Entertainment Demos

- **Chess Player** - Strategic piece movements
- **Bartender** - Cocktail mixing choreography  
- **Musician** - Rhythmic conducting motions
- **Magician** - Magic trick performances
- **Cigar Roller** - Artisan crafting simulation

## ⚠️ Safety

- **🎮 SIMULATION ONLY** - Current scripts are for Gazebo simulation
- **🚨 REAL HARDWARE WARNING** - See [SIMULATION_VS_REAL.md](SIMULATION_VS_REAL.md) for critical safety info
- **Emergency Stop** - Press ESC or Ctrl+C anytime in simulation
- **Conservative Limits** - All movements use safe joint ranges
- **Real Hardware** - Requires special safety setup, training, and equipment

### 🚫 NEVER use simulation scripts on real robot without proper safety measures!

## 🤝 Contributing

1. Fork the repository
2. Create feature branch: `git checkout -b feature-name`
3. Commit changes: `git commit -am 'Add feature'`
4. Push branch: `git push origin feature-name`
5. Submit pull request

## 📄 License

This project contains code from Unitree Robotics. Please respect their licensing terms.

## 🔗 Links

- [Unitree Z1 Documentation](https://support.unitree.com/home/en/Z1_developer)
- [ROS Noetic Documentation](http://wiki.ros.org/noetic)
- [Gazebo Tutorials](http://gazebosim.org/tutorials)

## 📞 Support

For issues:
1. Check [README_SCRIPTS.md](README_SCRIPTS.md) for troubleshooting
2. Review [SETUP_UBUNTU_20.04.md](SETUP_UBUNTU_20.04.md) for manual setup
3. Open GitHub issue with error details

---

**Made with ❤️ for robotics enthusiasts**