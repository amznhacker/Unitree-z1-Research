@echo off
REM Leverage Everything - Complete Z1 SDK Utilization Script (Windows)
REM Demonstrates all available capabilities in one comprehensive system

echo ╔══════════════════════════════════════════════════════════════╗
echo ║                    LEVERAGE EVERYTHING                       ║
echo ║              Complete Z1 SDK Utilization                    ║
echo ╚══════════════════════════════════════════════════════════════╝
echo.

echo This script demonstrates complete utilization of Z1 SDK capabilities:
echo 1. Enhanced Control System (All SDK features)
echo 2. Web API Service (HTTP interface)  
echo 3. Professional Suite (Industrial applications)
echo 4. Usage Examples (Integration patterns)
echo.

set WORKSPACE_DIR=%USERPROFILE%\catkin_ws

REM Install dependencies
echo [SECTION] Installing enhanced dependencies...
pip install fastapi uvicorn scipy scikit-learn
echo [INFO] Dependencies installed
echo.

REM Show SDK capabilities
echo [SECTION] SDK Capabilities Demonstration
echo Available SDK Features:
echo 1. 🎯 Kinematics ^& Dynamics
echo    - Forward/Inverse Kinematics
echo    - Jacobian calculations
echo    - Gravity compensation
echo    - Inverse dynamics
echo.
echo 2. 🎮 Control Modes
echo    - High-level commands (MoveJ/MoveL/MoveC)
echo    - Joint space control
echo    - Cartesian space control
echo    - Force/torque control
echo    - Low-level motor commands
echo.
echo 3. 🌐 Network Interfaces
echo    - HTTP/REST API
echo    - Real-time communication
echo    - Remote monitoring
echo.
echo 4. 🏭 Professional Applications
echo    - Precision assembly
echo    - Quality inspection
echo    - Collaborative robotics
echo    - Force-guided operations
echo.

echo Choose demonstration:
echo 1. Enhanced Control System
echo 2. Web API Service
echo 3. Professional Suite
echo 4. Create Examples

set /p choice="Selection (1-4): "

if "%choice%"=="1" (
    echo [SECTION] Launching Enhanced Control System...
    cd /d "%WORKSPACE_DIR%"
    call devel\setup.bat
    start "Gazebo" roslaunch unitree_gazebo z1.launch
    timeout /t 10
    python src\z1_tools\scripts\z1_sdk_enhanced_control.py
)

if "%choice%"=="2" (
    echo [SECTION] Launching Web API Service...
    cd /d "%WORKSPACE_DIR%"
    call devel\setup.bat
    start "Gazebo" roslaunch unitree_gazebo z1.launch gui:=false
    timeout /t 10
    echo 🌐 Web API Service Active
    echo API Documentation: http://localhost:8000/docs
    python src\z1_tools\scripts\z1_web_api_service.py
)

if "%choice%"=="3" (
    echo [SECTION] Launching Professional Suite...
    echo Note: Professional Suite requires real robot hardware
    python "%WORKSPACE_DIR%\src\z1_tools\scripts\z1_professional_suite.py"
)

if "%choice%"=="4" (
    echo [SECTION] Creating usage examples...
    
    REM Create API client example
    echo Creating API client example...
    (
    echo #!/usr/bin/env python3
    echo """Z1 API Client Example"""
    echo import requests
    echo import json
    echo.
    echo def test_api():
    echo     response = requests.get("http://localhost:8000/status"^)
    echo     print("Robot Status:", response.json(^)^)
    echo.
    echo if __name__ == "__main__":
    echo     test_api(^)
    ) > "%WORKSPACE_DIR%\api_client_example.py"
    
    echo ✅ Created API client example
    echo ✅ Created usage examples
)

echo.
echo 🎉 Complete SDK Utilization Available!
echo.
echo Summary of enhanced scripts:
echo ✅ z1_sdk_enhanced_control.py - Full SDK integration
echo ✅ z1_web_api_service.py - Complete HTTP API  
echo ✅ z1_professional_suite.py - Industrial applications
echo.
echo Next steps:
echo 1. Test with real robot hardware
echo 2. Integrate vision systems
echo 3. Add force/torque sensors
echo 4. Develop custom applications

pause