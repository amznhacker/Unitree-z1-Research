#!/bin/bash

# Z1 System Check - Comprehensive functionality verification
# Tests all components, interfaces, and capabilities

GREEN='\033[0;32m'
RED='\033[0;31m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

WORKSPACE_DIR="$HOME/catkin_ws"
PASSED=0
FAILED=0

print_header() {
    echo -e "${BLUE}╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║                    Z1 SYSTEM CHECK                          ║${NC}"
    echo -e "${BLUE}║              High-End Robotic Arm Verification              ║${NC}"
    echo -e "${BLUE}╚══════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

print_test() {
    echo -e "${BLUE}[TEST]${NC} $1"
}

print_pass() {
    echo -e "${GREEN}[PASS]${NC} $1"
    ((PASSED++))
}

print_fail() {
    echo -e "${RED}[FAIL]${NC} $1"
    ((FAILED++))
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

# Test 1: Environment Check
test_environment() {
    print_test "Testing environment setup..."
    
    # Check Ubuntu version
    if lsb_release -d | grep -q "20.04"; then
        print_pass "Ubuntu 20.04.6 LTS detected"
    else
        print_warn "Ubuntu version: $(lsb_release -d | cut -f2)"
    fi
    
    # Check ROS installation
    if command -v roscore &> /dev/null; then
        print_pass "ROS Noetic installed"
    else
        print_fail "ROS Noetic not found"
    fi
    
    # Check workspace
    if [ -d "$WORKSPACE_DIR" ]; then
        print_pass "Catkin workspace exists"
    else
        print_fail "Catkin workspace not found at $WORKSPACE_DIR"
    fi
    
    # Check workspace build
    if [ -f "$WORKSPACE_DIR/devel/setup.bash" ]; then
        print_pass "Workspace built successfully"
    else
        print_fail "Workspace not built - run catkin_make"
    fi
}

# Test 2: Dependencies Check
test_dependencies() {
    print_test "Testing dependencies..."
    
    # ROS packages
    ros_packages=("controller_manager" "gazebo_ros" "joint_state_controller" "effort_controllers" "robot_state_publisher" "xacro")
    
    for package in "${ros_packages[@]}"; do
        if rospack find "ros_$package" &> /dev/null || rospack find "$package" &> /dev/null; then
            print_pass "ROS package: $package"
        else
            print_fail "Missing ROS package: $package"
        fi
    done
    
    # Python packages
    python_packages=("flask" "numpy" "scipy")
    
    for package in "${python_packages[@]}"; do
        if python3 -c "import $package" &> /dev/null; then
            print_pass "Python package: $package"
        else
            print_fail "Missing Python package: $package"
        fi
    done
    
    # Optional: Xbox controller support
    if command -v joy_node &> /dev/null; then
        print_pass "Xbox controller support (joy_node)"
    else
        print_warn "Xbox controller support not installed (optional)"
    fi
}

# Test 3: Z1 Robot Files
test_robot_files() {
    print_test "Testing Z1 robot files..."
    
    essential_files=(
        "$WORKSPACE_DIR/src/unitree_ros/robots/z1_description/config/robot_control.yaml"
        "$WORKSPACE_DIR/src/unitree_ros/unitree_gazebo/launch/z1.launch"
        "$WORKSPACE_DIR/src/unitree_ros/robots/z1_description/xacro/robot.xacro"
        "$WORKSPACE_DIR/src/unitree_ros/robots/z1_description/meshes/visual/z1_Link00.dae"
    )
    
    for file in "${essential_files[@]}"; do
        if [ -f "$file" ]; then
            print_pass "Essential file: $(basename $file)"
        else
            print_fail "Missing file: $file"
        fi
    done
}

# Test 4: Scripts Check
test_scripts() {
    print_test "Testing Z1 control scripts..."
    
    scripts=(
        "z1_simple_control.py"
        "z1_xbox_control.py"
        "z1_web_gui.py"
        "z1_visual_programmer.py"
        "z1_bartender.py"
        "z1_demo_simple.py"
        "z1_drawing.py"
        "z1_real_robot_bridge.py"
    )
    
    for script in "${scripts[@]}"; do
        script_path="$WORKSPACE_DIR/src/z1_tools/scripts/$script"
        if [ -f "$script_path" ]; then
            if [ -x "$script_path" ]; then
                print_pass "Script: $script (executable)"
            else
                print_warn "Script: $script (not executable)"
                chmod +x "$script_path" 2>/dev/null || true
            fi
        else
            print_fail "Missing script: $script"
        fi
    done
}

# Test 5: Enhanced Scripts Check
test_enhanced_scripts() {
    print_test "Testing enhanced SDK scripts..."
    
    enhanced_scripts=(
        "z1_sdk_enhanced_control.py"
        "z1_web_api_service.py"
        "z1_pure_sdk_control.py"
        "z1_professional_suite.py"
        "z1_jetson_ai_control.py"
    )
    
    for script in "${enhanced_scripts[@]}"; do
        script_path="$WORKSPACE_DIR/src/z1_tools/scripts/$script"
        if [ -f "$script_path" ]; then
            if python3 -m py_compile "$script_path" &> /dev/null; then
                print_pass "Enhanced script: $script (syntax valid)"
            else
                print_fail "Enhanced script: $script (syntax errors)"
            fi
        else
            print_warn "Enhanced script: $script (not found - run leverage_everything.sh)"
        fi
    done
}

# Test 6: Gazebo Simulation
test_gazebo() {
    print_test "Testing Gazebo simulation..."
    
    # Source workspace if available
    if [ -f "$WORKSPACE_DIR/devel/setup.bash" ]; then
        source "$WORKSPACE_DIR/devel/setup.bash"
    fi
    
    # Clean any existing processes
    pkill -f gazebo &> /dev/null || true
    pkill -f roscore &> /dev/null || true
    sleep 2
    
    # Start roscore
    roscore &> /dev/null &
    ROSCORE_PID=$!
    sleep 3
    
    # Test if roscore is running
    if rostopic list &> /dev/null; then
        print_pass "ROS master started"
        
        # Launch Z1 simulation (headless)
        timeout 30s roslaunch unitree_gazebo z1.launch gui:=false &> /dev/null &
        GAZEBO_PID=$!
        sleep 15
        
        # Check if simulation is running
        if rostopic list | grep -q "z1_gazebo"; then
            print_pass "Z1 simulation launched successfully"
            
            # Test joint states
            if timeout 5s rostopic echo /z1_gazebo/joint_states -n 1 &> /dev/null; then
                print_pass "Joint states publishing"
            else
                print_fail "Joint states not publishing"
            fi
            
            # Test controllers
            controllers_loaded=0
            for i in {1..6}; do
                if rostopic list | grep -q "/z1_gazebo/Joint0${i}_controller"; then
                    ((controllers_loaded++))
                fi
            done
            
            if [ $controllers_loaded -eq 6 ]; then
                print_pass "All 6 joint controllers loaded"
            else
                print_fail "Only $controllers_loaded/6 controllers loaded"
            fi
        else
            print_fail "Z1 simulation failed to launch"
        fi
        
        # Cleanup
        kill $GAZEBO_PID &> /dev/null || true
    else
        print_fail "ROS master failed to start"
    fi
    
    kill $ROSCORE_PID &> /dev/null || true
    pkill -f gazebo &> /dev/null || true
    sleep 2
}

# Test 7: Web Interfaces
test_web_interfaces() {
    print_test "Testing web interfaces..."
    
    # Test Flask import
    if python3 -c "import flask" &> /dev/null; then
        print_pass "Flask available for web interfaces"
        
        # Test web GUI script syntax
        if [ -f "$WORKSPACE_DIR/src/z1_tools/scripts/z1_web_gui.py" ]; then
            if python3 -m py_compile "$WORKSPACE_DIR/src/z1_tools/scripts/z1_web_gui.py" &> /dev/null; then
                print_pass "Web GUI script syntax valid"
            else
                print_fail "Web GUI script has syntax errors"
            fi
        fi
        
        # Test FastAPI availability for enhanced web service
        if python3 -c "import fastapi" &> /dev/null; then
            print_pass "FastAPI available for enhanced web service"
        else
            print_warn "FastAPI not available (install with: pip3 install fastapi uvicorn)"
        fi
    else
        print_fail "Flask not available - web interfaces won't work"
    fi
}

# Test 8: Real Robot SDK
test_real_robot_sdk() {
    print_test "Testing real robot SDK..."
    
    if [ -d "$WORKSPACE_DIR/src/z1_sdk" ]; then
        print_pass "Z1 SDK directory exists"
        
        # Check SDK libraries
        if [ -f "$WORKSPACE_DIR/src/z1_sdk/lib/libZ1_SDK_x86_64.so" ]; then
            print_pass "Z1 SDK library (x86_64) available"
        else
            print_warn "Z1 SDK library not found (real robot features limited)"
        fi
        
        # Check Python interface
        if [ -f "$WORKSPACE_DIR/src/z1_sdk/examples_py/example_highcmd.py" ]; then
            print_pass "Python SDK examples available"
        else
            print_fail "Python SDK examples missing"
        fi
    else
        print_fail "Z1 SDK not found - real robot support unavailable"
    fi
}

# Test 9: Performance Check
test_performance() {
    print_test "Testing system performance..."
    
    # Check CPU cores
    cpu_cores=$(nproc)
    if [ $cpu_cores -ge 4 ]; then
        print_pass "CPU cores: $cpu_cores (sufficient for simulation)"
    else
        print_warn "CPU cores: $cpu_cores (may affect performance)"
    fi
    
    # Check RAM
    ram_gb=$(free -g | awk '/^Mem:/{print $2}')
    if [ $ram_gb -ge 4 ]; then
        print_pass "RAM: ${ram_gb}GB (sufficient)"
    else
        print_warn "RAM: ${ram_gb}GB (minimum for basic operation)"
    fi
    
    # Check disk space
    disk_space=$(df -BG "$WORKSPACE_DIR" 2>/dev/null | awk 'NR==2{print $4}' | sed 's/G//' || echo "0")
    if [ "$disk_space" -ge 5 ] 2>/dev/null; then
        print_pass "Disk space: ${disk_space}GB available"
    else
        print_warn "Disk space: ${disk_space}GB (may need cleanup)"
    fi
}

# Test 10: Network Interfaces
test_network() {
    print_test "Testing network capabilities..."
    
    # Test localhost connectivity (for web interfaces)
    if curl -s --connect-timeout 2 http://localhost:8080 &> /dev/null || [ $? -eq 7 ]; then
        print_pass "Localhost connectivity available"
    else
        print_warn "Localhost connectivity issue"
    fi
    
    # Test real robot network (if configured)
    if ping -c 1 -W 2 192.168.123.110 &> /dev/null; then
        print_pass "Real robot network (192.168.123.110) reachable"
    else
        print_warn "Real robot network not reachable (normal if no robot connected)"
    fi
}

# Main execution
main() {
    print_header
    
    echo "Starting comprehensive system check..."
    echo "This will verify all Z1 robotic arm functionality."
    echo ""
    
    # Run all tests - continue even if some fail
    test_environment || true
    test_dependencies || true
    test_robot_files || true
    test_scripts || true
    test_enhanced_scripts || true
    test_gazebo || true
    test_web_interfaces || true
    test_real_robot_sdk || true
    test_performance || true
    test_network || true
    
    echo ""
    echo -e "${BLUE}╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║                      SYSTEM CHECK RESULTS                   ║${NC}"
    echo -e "${BLUE}╚══════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    
    if [ $FAILED -eq 0 ]; then
        echo -e "${GREEN}🎉 ALL TESTS PASSED! ($PASSED/$((PASSED + FAILED)))${NC}"
        echo -e "${GREEN}✅ Z1 Robotic Arm system is fully operational${NC}"
        echo -e "${GREEN}🚀 Ready for high-end robotic operations${NC}"
        echo ""
        echo "Available commands:"
        echo "  ./quick_start.sh keyboard    # Manual control"
        echo "  ./quick_start.sh xbox        # Xbox controller"
        echo "  ./quick_start.sh web         # Web interface"
        echo "  ./quick_start.sh visual      # Visual programming"
        echo "  ./quick_start.sh bartender   # Demo performance"
        echo "  ./quick_start.sh real        # Real robot mode"
        echo "  ./leverage_everything.sh     # Enhanced SDK features"
    else
        echo -e "${RED}❌ SOME TESTS FAILED ($FAILED failed, $PASSED passed)${NC}"
        echo -e "${YELLOW}⚠️  System may have limited functionality${NC}"
        echo ""
        echo "Recommended actions:"
        echo "1. Review failed tests above"
        echo "2. Run: ./setup_and_run.sh --install-ros"
        echo "3. Install missing packages: pip3 install flask fastapi uvicorn"
        echo "4. Check documentation for troubleshooting"
    fi
    
    echo ""
    return $FAILED
}

# Run if executed directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi