#!/bin/bash
# Setup Xbox Controller for Z1 Control

echo "🎮 Setting up Xbox Controller for Z1 Robotic Arm"
echo "================================================"

# Check if running on Linux
if [[ "$OSTYPE" != "linux-gnu"* ]]; then
    echo "❌ This setup is for Linux systems only"
    exit 1
fi

# Install required ROS packages
echo "📦 Installing ROS joy package..."
sudo apt-get update
sudo apt-get install -y ros-noetic-joy ros-noetic-joystick-drivers

# Check if Xbox controller is connected
echo "🔍 Checking for Xbox controller..."
if lsusb | grep -i "xbox\|microsoft"; then
    echo "✅ Xbox controller detected!"
else
    echo "⚠️  No Xbox controller detected. Please connect your controller."
fi

# Check joystick device
echo "🕹️  Checking joystick devices..."
if ls /dev/input/js* 2>/dev/null; then
    echo "✅ Joystick device found:"
    ls -la /dev/input/js*
    
    # Test joystick
    echo "🧪 Testing joystick (press Ctrl+C to stop)..."
    echo "Move your controller sticks and press buttons to test..."
    timeout 10s jstest /dev/input/js0 || echo "⏰ Test timeout - controller should be working"
else
    echo "❌ No joystick device found at /dev/input/js*"
    echo "💡 Try unplugging and reconnecting your Xbox controller"
fi

# Create launch file for easy startup
echo "📝 Creating launch file..."
cat > ../launch/z1_xbox_control.launch << 'EOF'
<launch>
  <!-- Xbox Controller Node -->
  <node name="joy_node" pkg="joy" type="joy_node" output="screen">
    <param name="dev" value="/dev/input/js0" />
    <param name="deadzone" value="0.1" />
    <param name="autorepeat_rate" value="50" />
  </node>
  
  <!-- Z1 Xbox Control -->
  <node name="z1_xbox_control" pkg="z1_tools" type="z1_xbox_control.py" output="screen">
    <param name="mode" value="10" />
    <param name="kp" value="35.0" />
    <param name="kd" value="1.5" />
  </node>
</launch>
EOF

echo "✅ Launch file created: ../launch/z1_xbox_control.launch"

# Create udev rule for Xbox controller permissions
echo "🔐 Setting up controller permissions..."
sudo tee /etc/udev/rules.d/99-xbox-controller.rules > /dev/null << 'EOF'
# Xbox Controller udev rules
SUBSYSTEM=="input", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="028e", MODE="0666"
SUBSYSTEM=="input", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="02d1", MODE="0666"
SUBSYSTEM=="input", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="02dd", MODE="0666"
SUBSYSTEM=="input", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="02e3", MODE="0666"
SUBSYSTEM=="input", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="02ea", MODE="0666"
SUBSYSTEM=="input", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="0719", MODE="0666"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger

echo "✅ Controller permissions configured"

# Test ROS joy node
echo "🧪 Testing ROS joy node..."
echo "Starting joy node for 5 seconds..."
timeout 5s rosrun joy joy_node &
sleep 2

if rostopic list | grep -q "/joy"; then
    echo "✅ ROS joy node working!"
    echo "📊 Joy topic data (move controller to see output):"
    timeout 3s rostopic echo /joy -n 1 2>/dev/null || echo "⚠️  No controller input detected"
else
    echo "❌ ROS joy node not working properly"
fi

echo ""
echo "🎮 Xbox Controller Setup Complete!"
echo "=================================="
echo ""
echo "📋 Usage Instructions:"
echo "1. Connect your Xbox controller (wired or wireless)"
echo "2. Launch Z1 in Gazebo: roslaunch unitree_gazebo z1.launch"
echo "3. Start Xbox control: roslaunch z1_tools z1_xbox_control.launch"
echo ""
echo "🎯 Controller Mapping:"
echo "  Left Stick:    Base rotation + Shoulder pitch"
echo "  Right Stick:   Elbow + Wrist pitch"
echo "  LT/RT:         Forearm roll"
echo "  LB/RB:         Wrist roll"
echo "  A/B:           Gripper close/open"
echo "  X:             Precision mode toggle"
echo "  Y:             Emergency stop"
echo "  Start/Back:    Speed up/down"
echo ""
echo "🔧 Troubleshooting:"
echo "  - If controller not detected: sudo chmod 666 /dev/input/js0"
echo "  - Test controller: jstest /dev/input/js0"
echo "  - Check ROS topics: rostopic list | grep joy"
echo ""
echo "✨ Happy controlling!"