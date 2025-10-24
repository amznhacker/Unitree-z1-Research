# Mouse/Mudra Band Control Guide

## Overview
Control the Unitree Z1 robot using mouse movements, clicks, and scroll wheel. Perfect for Mudra band integration.

## Quick Start
```bash
# Install pygame dependency
pip3 install pygame

# Start mouse control
./quick_start.sh mouse
```

## Control Mapping

| Input | Robot Action |
|-------|--------------|
| **Mouse X Movement** | Base joint rotation (left/right) |
| **Mouse Y Movement** | Shoulder joint movement (up/down) |
| **Left Click** | Open gripper |
| **Right Click** | Close gripper |
| **Scroll Wheel Up** | Extend elbow |
| **Scroll Wheel Down** | Bend elbow |
| **ESC Key** | Emergency stop and quit |

## Features
- **Real-time Control** - Immediate response to mouse input
- **Safe Limits** - Automatic joint limit enforcement
- **Emergency Stop** - ESC key for instant shutdown
- **Visual Feedback** - Control window shows active status
- **Mudra Compatible** - Works with any mouse-compatible device

## Technical Details
- **Sensitivity**: 0.002 radians per pixel movement
- **Update Rate**: 30 Hz for smooth control
- **Joint Limits**: Conservative safety limits enforced
- **Control Window**: 400x300 pixels (minimal resource usage)

## Mudra Band Setup
1. Pair Mudra band with computer via Bluetooth
2. Ensure it's recognized as a standard mouse device
3. Launch mouse control: `./quick_start.sh mouse`
4. Use natural hand gestures to control the robot

## Troubleshooting
- **pygame not found**: Run `pip3 install pygame`
- **No response**: Check if control window has focus
- **Jerky movement**: Ensure stable mouse/Mudra connection
- **Emergency stop**: Press ESC key immediately

## Integration with Other Systems
The mouse control can be combined with:
- Real robot hardware (switch via Control Center)
- Web interface monitoring
- API logging and recording
- Custom gesture recognition systems