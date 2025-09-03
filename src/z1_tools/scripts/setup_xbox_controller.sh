#!/bin/bash

# Setup Xbox Controller for Z1 Robot Control on Ubuntu
echo "🎮 Setting up Xbox Controller for Z1 Robot..."

# Install ROS joy package
echo "📦 Installing ROS joy package..."
sudo apt update
sudo apt install -y ros-noetic-joy

# Check for connected controllers
echo "🔍 Checking for connected controllers..."
if ls /dev/input/js* 1> /dev/null 2>&1; then
    echo "✅ Controller(s) detected:"
    ls -la /dev/input/js*
else
    echo "❌ No controllers detected at /dev/input/js*"
    echo "💡 Connect your Xbox controller via USB or Bluetooth"
fi

# Test controller input
echo "🧪 Testing controller input..."
echo "Press Ctrl+C to stop the test"
echo "Move your controller sticks and buttons to see if they work:"

timeout 10s rostopic echo /joy -n 1 2>/dev/null || {
    echo "⚠️  Joy topic not available. Starting joy_node..."
    rosrun joy joy_node &
    JOY_PID=$!
    sleep 2
    
    echo "Testing again..."
    timeout 5s rostopic echo /joy -n 1 2>/dev/null && {
        echo "✅ Xbox controller is working!"
    } || {
        echo "❌ Xbox controller test failed"
        echo "💡 Troubleshooting:"
        echo "   1. Make sure controller is connected"
        echo "   2. Check permissions: sudo chmod 666 /dev/input/js0"
        echo "   3. Try different USB port"
        echo "   4. For Bluetooth: pair controller first"
    }
    
    kill $JOY_PID 2>/dev/null
}

echo ""
echo "🚀 Xbox controller setup complete!"
echo "📋 Usage:"
echo "   ./quick_start.sh xbox     # Use Xbox controller"
echo "   roslaunch z1_tools z1_xbox_control.launch  # Manual launch"
echo ""
echo "🎮 Xbox Controls:"
echo "   Left Stick: Base rotation & Shoulder pitch"
echo "   Right Stick: Elbow & Forearm roll"
echo "   D-Pad: Wrist pitch & roll"
echo "   RT/LT: Gripper open/close"
echo "   Back: Emergency stop"