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

## Enhanced Control System

### Control Modes (Switch with Scroll Wheel)
1. **Base & Shoulder Mode** (Default)
   - Mouse X → Base rotation (Joint01)
   - Mouse Y → Shoulder movement (Joint02)

2. **Elbow & Forearm Mode**
   - Mouse X → Elbow bend/extend (Joint03)
   - Mouse Y → Forearm rotation (Joint04)

3. **Wrist Pitch & Roll Mode**
   - Mouse X → Wrist pitch (Joint05)
   - Mouse Y → Wrist roll (Joint06)

### Universal Controls
| Input | Robot Action |
|-------|--------------|
| **Scroll Wheel** | Switch control modes |
| **Left Click** | Open gripper |
| **Right Click** | Close gripper |
| **ESC Key** | Emergency stop and quit |

## Features
- **Full 6-DOF Control** - All robot joints accessible
- **Mode Switching** - Scroll wheel changes control pairs
- **Real-time Control** - Immediate response to mouse input
- **Safe Limits** - Automatic joint limit enforcement
- **Visual Interface** - Shows current mode and joint positions
- **Emergency Stop** - ESC key for instant shutdown
- **Mudra Compatible** - Works with any mouse-compatible device

## Technical Details
- **Sensitivity**: 0.003 radians per pixel movement
- **Update Rate**: 30 Hz for smooth control
- **Joint Limits**: Conservative safety limits enforced
- **Control Window**: 600x400 pixels with status display
- **Gripper Step**: 0.1 units per click

## Control Workflow
1. **Start**: Default mode controls base and shoulder
2. **Switch Modes**: Scroll wheel cycles through joint pairs
3. **Fine Control**: Mouse movements control active joints
4. **Gripper**: Left/right clicks open/close gripper
5. **Emergency**: ESC key stops all movement

## Mudra Band Setup
1. Pair Mudra band with computer via Bluetooth
2. Ensure it's recognized as a standard mouse device
3. Launch enhanced mouse control: `./quick_start.sh mouse`
4. Use scroll gesture to switch between joint control modes
5. Use natural hand movements to control active joint pair
6. Use tap gestures for gripper control

## Troubleshooting
- **pygame not found**: Run `pip3 install pygame`
- **No response**: Check if control window has focus
- **Jerky movement**: Ensure stable mouse/Mudra connection
- **Emergency stop**: Press ESC key immediately

## Integration with Other Systems
The enhanced mouse control can be combined with:
- Real robot hardware (switch via Control Center)
- Web interface monitoring
- API logging and recording
- Custom gesture recognition systems
- Multi-modal control (voice + mouse)