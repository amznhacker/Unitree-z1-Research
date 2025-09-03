#!/bin/bash

# Leverage Everything - Complete Z1 SDK Utilization Script
# Demonstrates all available capabilities in one comprehensive system

set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

WORKSPACE_DIR="$HOME/catkin_ws"

print_header() {
    echo -e "${BLUE}╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║                    LEVERAGE EVERYTHING                       ║${NC}"
    echo -e "${BLUE}║              Complete Z1 SDK Utilization                    ║${NC}"
    echo -e "${BLUE}╚══════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

print_section() {
    echo -e "${YELLOW}[SECTION]${NC} $1"
}

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

# Make scripts executable
setup_permissions() {
    print_section "Setting up script permissions..."
    
    scripts=(
        "z1_sdk_enhanced_control.py"
        "z1_web_api_service.py" 
        "z1_professional_suite.py"
    )
    
    for script in "${scripts[@]}"; do
        script_path="$WORKSPACE_DIR/src/z1_tools/scripts/$script"
        if [ -f "$script_path" ]; then
            chmod +x "$script_path"
            print_info "Made $script executable"
        fi
    done
}

# Install additional dependencies
install_dependencies() {
    print_section "Installing enhanced dependencies..."
    
    # FastAPI for web service
    pip3 install fastapi uvicorn --user
    
    # Additional scientific computing
    pip3 install scipy scikit-learn --user
    
    print_info "Dependencies installed"
}

# Demonstrate SDK capabilities
demo_sdk_capabilities() {
    print_section "SDK Capabilities Demonstration"
    
    echo "Available SDK Features:"
    echo "1. 🎯 Kinematics & Dynamics"
    echo "   - Forward/Inverse Kinematics"
    echo "   - Jacobian calculations"
    echo "   - Gravity compensation"
    echo "   - Inverse dynamics"
    echo ""
    
    echo "2. 🎮 Control Modes"
    echo "   - High-level commands (MoveJ/MoveL/MoveC)"
    echo "   - Joint space control"
    echo "   - Cartesian space control"
    echo "   - Force/torque control"
    echo "   - Low-level motor commands"
    echo ""
    
    echo "3. 🌐 Network Interfaces"
    echo "   - HTTP/REST API"
    echo "   - Real-time communication"
    echo "   - Remote monitoring"
    echo ""
    
    echo "4. 🏭 Professional Applications"
    echo "   - Precision assembly"
    echo "   - Quality inspection"
    echo "   - Collaborative robotics"
    echo "   - Force-guided operations"
    echo ""
}

# Launch enhanced control system
launch_enhanced_control() {
    print_section "Launching Enhanced Control System..."
    
    cd "$WORKSPACE_DIR"
    source devel/setup.bash
    
    # Start simulation
    print_info "Starting Gazebo simulation..."
    roslaunch unitree_gazebo z1.launch gui:=true &
    GAZEBO_PID=$!
    sleep 10
    
    # Launch enhanced control
    print_info "Starting SDK Enhanced Control..."
    rosrun z1_tools z1_sdk_enhanced_control.py &
    CONTROL_PID=$!
    
    echo ""
    echo -e "${GREEN}🚀 Enhanced Control System Active${NC}"
    echo "Available commands in control interface:"
    echo "  d = Full SDK demonstration"
    echo "  h = High-level commands (MoveJ/MoveL/MoveC)"
    echo "  l = Low-level torque control"
    echo "  f = Force control demonstration"
    echo "  i = Impedance control"
    echo ""
    
    # Wait for user input
    read -p "Press Enter to continue to next demo..."
    
    # Cleanup
    kill $CONTROL_PID $GAZEBO_PID 2>/dev/null || true
    pkill -f gazebo 2>/dev/null || true
    sleep 2
}

# Launch web API service
launch_web_api() {
    print_section "Launching Web API Service..."
    
    cd "$WORKSPACE_DIR"
    source devel/setup.bash
    
    # Start simulation
    print_info "Starting Gazebo simulation..."
    roslaunch unitree_gazebo z1.launch gui:=false &
    GAZEBO_PID=$!
    sleep 10
    
    # Launch web API
    print_info "Starting Web API Service..."
    python3 src/z1_tools/scripts/z1_web_api_service.py &
    API_PID=$!
    sleep 3
    
    echo ""
    echo -e "${GREEN}🌐 Web API Service Active${NC}"
    echo "API Documentation: http://localhost:8000/docs"
    echo "API Endpoints:"
    echo "  GET  /status              - Robot status"
    echo "  POST /move/joint          - Joint movement"
    echo "  POST /move/cartesian      - Cartesian movement"
    echo "  POST /kinematics/forward  - Forward kinematics"
    echo "  POST /kinematics/inverse  - Inverse kinematics"
    echo "  POST /control/force       - Force control"
    echo "  POST /trajectory/execute  - Trajectory execution"
    echo ""
    
    # Test API
    print_info "Testing API endpoints..."
    
    # Test status
    if command -v curl &> /dev/null; then
        echo "Testing /status endpoint:"
        curl -s http://localhost:8000/status | python3 -m json.tool || echo "API not ready yet"
        echo ""
        
        echo "Testing /capabilities endpoint:"
        curl -s http://localhost:8000/capabilities | python3 -m json.tool || echo "API not ready yet"
    fi
    
    read -p "Press Enter to continue to next demo..."
    
    # Cleanup
    kill $API_PID $GAZEBO_PID 2>/dev/null || true
    pkill -f gazebo 2>/dev/null || true
    sleep 2
}

# Launch professional suite
launch_professional_suite() {
    print_section "Launching Professional Suite..."
    
    echo -e "${YELLOW}Note: Professional Suite requires real robot hardware${NC}"
    echo "This demo shows the interface for:"
    echo "  1. Precision Pick & Place"
    echo "  2. Assembly Operations"
    echo "  3. Quality Inspection"
    echo "  4. Collaborative Robotics"
    echo ""
    
    # Show interface without real robot
    python3 "$WORKSPACE_DIR/src/z1_tools/scripts/z1_professional_suite.py" || true
}

# Create usage examples
create_usage_examples() {
    print_section "Creating usage examples..."
    
    # Python client example
    cat > "$WORKSPACE_DIR/api_client_example.py" << 'EOF'
#!/usr/bin/env python3
"""
Z1 API Client Example - How to use the web API
"""

import requests
import json
import time

API_BASE = "http://localhost:8000"

def test_api():
    # Get robot status
    response = requests.get(f"{API_BASE}/status")
    print("Robot Status:", response.json())
    
    # Move joints
    joint_cmd = {
        "joints": [0.0, 0.5, -0.5, 0.0, 0.0, 0.0],
        "speed": 1.0
    }
    response = requests.post(f"{API_BASE}/move/joint", json=joint_cmd)
    print("Joint Movement:", response.json())
    
    # Forward kinematics
    response = requests.post(f"{API_BASE}/kinematics/forward", json=joint_cmd)
    print("Forward Kinematics:", response.json())
    
    # Cartesian movement
    cartesian_cmd = {
        "position": [0.3, 0.0, 0.4, 0.0, 0.0, 0.0],
        "gripper": 0.0,
        "speed": 0.5
    }
    response = requests.post(f"{API_BASE}/move/cartesian", json=cartesian_cmd)
    print("Cartesian Movement:", response.json())

if __name__ == "__main__":
    test_api()
EOF
    
    chmod +x "$WORKSPACE_DIR/api_client_example.py"
    print_info "Created API client example: $WORKSPACE_DIR/api_client_example.py"
    
    # ROS integration example
    cat > "$WORKSPACE_DIR/ros_integration_example.py" << 'EOF'
#!/usr/bin/env python3
"""
ROS Integration Example - Combine ROS with SDK
"""

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray

def pose_callback(msg):
    """Handle pose commands from ROS"""
    print(f"Received pose: {msg.pose.position}")
    # Convert to SDK commands here

def main():
    rospy.init_node("ros_sdk_integration")
    
    # Subscribe to ROS topics
    rospy.Subscriber("/move_group/goal", PoseStamped, pose_callback)
    
    # Publish SDK status to ROS
    status_pub = rospy.Publisher("/z1_sdk/status", Float64MultiArray, queue_size=1)
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Publish robot status
        status_msg = Float64MultiArray()
        status_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Joint positions
        status_pub.publish(status_msg)
        
        rate.sleep()

if __name__ == "__main__":
    main()
EOF
    
    chmod +x "$WORKSPACE_DIR/ros_integration_example.py"
    print_info "Created ROS integration example: $WORKSPACE_DIR/ros_integration_example.py"
}

# Main execution
main() {
    print_header
    
    echo "This script demonstrates complete utilization of Z1 SDK capabilities:"
    echo "1. Enhanced Control System (All SDK features)"
    echo "2. Web API Service (HTTP interface)"
    echo "3. Professional Suite (Industrial applications)"
    echo "4. Usage Examples (Integration patterns)"
    echo ""
    
    setup_permissions
    install_dependencies
    demo_sdk_capabilities
    
    echo -e "${BLUE}Choose demonstration:${NC}"
    echo "1. Enhanced Control System"
    echo "2. Web API Service"
    echo "3. Professional Suite"
    echo "4. All Demos (sequential)"
    echo "5. Create Examples Only"
    
    read -p "Selection (1-5): " choice
    
    case $choice in
        1)
            launch_enhanced_control
            ;;
        2)
            launch_web_api
            ;;
        3)
            launch_professional_suite
            ;;
        4)
            launch_enhanced_control
            launch_web_api
            launch_professional_suite
            ;;
        5)
            create_usage_examples
            ;;
        *)
            echo "Invalid selection"
            exit 1
            ;;
    esac
    
    create_usage_examples
    
    echo ""
    echo -e "${GREEN}🎉 Complete SDK Utilization Demonstration Complete!${NC}"
    echo ""
    echo "Summary of created enhancements:"
    echo "✅ z1_sdk_enhanced_control.py - Full SDK integration"
    echo "✅ z1_web_api_service.py - Complete HTTP API"
    echo "✅ z1_professional_suite.py - Industrial applications"
    echo "✅ api_client_example.py - Usage examples"
    echo "✅ ros_integration_example.py - ROS integration"
    echo ""
    echo "Next steps:"
    echo "1. Test with real robot hardware"
    echo "2. Integrate vision systems"
    echo "3. Add force/torque sensors"
    echo "4. Develop custom applications"
}

# Run if executed directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi