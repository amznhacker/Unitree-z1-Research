# Unitree Z1 Robotic Arm - Complete Workspace

Complete ROS workspace for Unitree Z1 robotic arm simulation and real robot control on Ubuntu 20.04.6 LTS.

## 🚀 Quick Start

### First Time Setup (One Command)
```bash
git clone git@github.com:amznhacker/Unitree-z1-Research.git
cd Unitree-z1-Research
chmod +x setup_and_run.sh
./setup_and_run.sh --install-ros
```

### Daily Use (One Command)
```bash
./quick_start.sh keyboard    # Simulation with keyboard
./quick_start.sh real        # Connect to real robot
```

## 🎮 Control Methods

| Method | Command | Description |
|--------|---------|-------------|
| **Keyboard** | `./quick_start.sh keyboard` | WASD movement, intuitive controls |
| **Xbox Controller** | `./quick_start.sh xbox` | Gamepad control |
| **Pick & Place Demo** | `./quick_start.sh demo` | Automated demonstration |
| **Drawing Demo** | `./quick_start.sh draw` | Geometric patterns |
| **Real Robot** | `./quick_start.sh real` | Connect to physical Z1 |

## 🎯 Keyboard Controls

- **WASD**: Base rotation & shoulder pitch
- **ZE**: Elbow bend/extend  
- **RF**: Forearm roll
- **TG**: Wrist pitch
- **YH**: Wrist roll
- **Space**: Open gripper
- **X**: Close gripper
- **ESC**: Emergency stop

## 🤖 Two Modes

### 🎮 **Simulation Mode (Default)**
- Safe learning environment
- No hardware required
- Perfect for development
- Gazebo physics simulation

### 🔧 **Real Robot Mode**
- Connects to physical Z1 arm
- Requires robot setup and safety training
- Uses same controls as simulation
- Automatic safety limits

## 🛠 Requirements

- **Ubuntu 20.04.6 LTS**
- **4GB+ RAM, 20GB+ disk space**
- **Internet connection** (initial setup)
- **Real Z1 Robot** (optional, for real mode)

## 📋 Project Structure

```
Unitree-z1-Research/
├── src/
│   ├── unitree_ros/          # Robot descriptions & Gazebo
│   ├── z1_sdk/               # Real robot SDK & examples
│   ├── z1_tools/             # Control scripts & demos
│   └── unitree_ros_to_real/  # Real hardware interface
├── setup_and_run.sh          # Complete setup script
├── quick_start.sh             # Daily use script
└── docs/                     # Documentation
```

## 🔧 Troubleshooting

### Gazebo won't start:
```bash
pkill -f gazebo && ./quick_start.sh
```

### Controllers not loading:
```bash
cd ~/catkin_ws && catkin_make clean && catkin_make
./quick_start.sh
```

### Real robot connection issues:
```bash
# Check robot IP and network connection
ping 192.168.123.110
```

## ⚠️ Safety

### **Simulation Mode:**
- ✅ Completely safe
- ✅ No physical risks
- ✅ Experiment freely

### **Real Robot Mode:**
- 🚨 **Physical robot - can cause injury**
- 🚨 **Requires safety training**
- 🚨 **Emergency stop must be accessible**
- 🚨 **Clear workspace required**

## 🚀 Advanced Usage

### **Custom Scripts:**
```bash
# Create your own control script
cp src/z1_tools/scripts/z1_simple_control.py my_control.py
# Edit and run: rosrun z1_tools my_control.py
```

### **Real Robot Setup:**
```bash
# 1. Connect robot via Ethernet
# 2. Configure network (192.168.123.x)
# 3. Test connection
./quick_start.sh real
```

### **SDK Examples:**
```bash
# C++ examples
cd src/z1_sdk/examples
make
./highcmd_basic

# Python examples  
cd src/z1_sdk/examples_py
python3 example_highcmd.py
```

## 🤝 Contributing

1. Fork the repository
2. Create feature branch: `git checkout -b feature-name`
3. Test in simulation first
4. Submit pull request

## 📄 License

This project contains code from Unitree Robotics. Please respect their licensing terms.

## 🔗 Links

- [Unitree Z1 Documentation](https://support.unitree.com/home/en/Z1_developer)
- [ROS Noetic Documentation](http://wiki.ros.org/noetic)
- [Gazebo Tutorials](http://gazebosim.org/tutorials)

## 📞 Support

For issues:
1. Check simulation works first: `./quick_start.sh keyboard`
2. See [TWO_TERMINAL_USAGE.md](TWO_TERMINAL_USAGE.md) for advanced usage
3. Review documentation for troubleshooting
4. Open GitHub issue with error details

---

**🎯 Perfect for learning robotics - Safe simulation with real robot capability!**