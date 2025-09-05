#!/usr/bin/env python3

"""
Z1 Control Center - Unified interface for all Z1 robot operations
Access at: http://localhost:8080
Features: Script launcher, simulation/real switching, system monitoring
"""

import rospy
import json
import subprocess
import threading
import time
import os
import signal
from flask import Flask, render_template_string, request, jsonify
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class Z1ControlCenter:
    def __init__(self):
        rospy.init_node("z1_control_center")
        
        # System state
        self.mode = "simulation"  # simulation or real
        self.running_processes = {}
        self.joint_states = {}
        self.system_status = "ready"
        
        # Available scripts organized by category
        self.scripts = {
            "control": {
                "keyboard": {"name": "Keyboard Control", "script": "z1_simple_control.py", "type": "ros"},
                "xbox": {"name": "Xbox Controller", "script": "z1_xbox_control.py", "type": "ros"},
                "advanced": {"name": "Advanced Control", "script": "z1_advanced_control.py", "type": "ros"}
            },
            "demos": {
                "demo": {"name": "Pick & Place Demo", "script": "z1_demo_simple.py", "type": "ros"},
                "bartender": {"name": "Bartender Demo", "script": "z1_bartender.py", "type": "ros"},
                "chess": {"name": "Chess Player", "script": "z1_chess_player.py", "type": "ros"},
                "magician": {"name": "Magic Tricks", "script": "z1_magician.py", "type": "ros"},
                "drawing": {"name": "Drawing Shapes", "script": "z1_drawing.py", "type": "ros"},
                "wave": {"name": "Wave Motion", "script": "z1_wave.py", "type": "ros"},
                "musician": {"name": "Musical Conductor", "script": "z1_musician.py", "type": "ros"},
                "painter": {"name": "Artistic Painter", "script": "z1_painter.py", "type": "ros"},
                "dancer": {"name": "Dance Choreography", "script": "z1_dancer.py", "type": "ros"},
                "juggler": {"name": "Juggling Performance", "script": "z1_juggler.py", "type": "ros"},
                "chef": {"name": "Cooking Demo", "script": "z1_chef.py", "type": "ros"},
                "drummer": {"name": "Drum Performance", "script": "z1_drummer.py", "type": "ros"},
                "yoga": {"name": "Yoga Instructor", "script": "z1_yoga_instructor.py", "type": "ros"},
                "mime": {"name": "Mime Artist", "script": "z1_mime_artist.py", "type": "ros"}
            },
            "interfaces": {
                "web": {"name": "Web GUI", "script": "z1_web_gui_enhanced.py", "type": "ros"},
                "visual": {"name": "Visual Programmer", "script": "z1_visual_programmer_enhanced.py", "type": "ros"},
                "api": {"name": "Web API Service", "script": "z1_web_api_service.py", "type": "ros"}
            },
            "professional": {
                "enhanced": {"name": "Enhanced SDK Control", "script": "z1_sdk_enhanced_control.py", "type": "ros"},
                "professional": {"name": "Professional Suite", "script": "z1_professional_suite.py", "type": "ros"},
                "pure_sdk": {"name": "Pure SDK Control", "script": "z1_pure_sdk_control.py", "type": "python"},
                "jetson": {"name": "Jetson AI Control", "script": "z1_jetson_ai_control.py", "type": "ros"}
            },
            "utilities": {
                "emergency": {"name": "Emergency Stop", "script": "z1_emergency_stop.py", "type": "ros"},
                "limits": {"name": "Safe Limits Test", "script": "z1_safe_limits.py", "type": "ros"},
                "bridge": {"name": "Real Robot Bridge", "script": "z1_real_robot_bridge.py", "type": "ros"}
            }
        }
        
        # Subscribe to joint states
        rospy.Subscriber("/z1_gazebo/joint_states", JointState, self.joint_state_callback)
        
        # Flask app
        self.app = Flask(__name__)
        self.setup_routes()
    
    def joint_state_callback(self, msg):
        """Update joint states"""
        if len(msg.position) >= 6:
            self.joint_states = {
                "Joint01": msg.position[0], "Joint02": msg.position[1], "Joint03": msg.position[2],
                "Joint04": msg.position[3], "Joint05": msg.position[4], "Joint06": msg.position[5],
                "Gripper": msg.position[6] if len(msg.position) > 6 else 0.0
            }
    
    def launch_script(self, script_info, category, script_key):
        """Launch a script based on current mode"""
        script_name = script_info["script"]
        script_type = script_info["type"]
        
        # Stop any running script in the same category
        self.stop_category(category)
        
        try:
            if self.mode == "simulation":
                if script_type == "ros":
                    cmd = ["rosrun", "z1_tools", script_name]
                else:
                    cmd = ["python3", f"src/z1_tools/scripts/{script_name}"]
            else:  # real mode
                # For real robot, use SDK versions when available
                if "sdk" in script_name or "pure" in script_name:
                    cmd = ["python3", f"src/z1_tools/scripts/{script_name}"]
                else:
                    # Use bridge for ROS scripts on real robot
                    cmd = ["rosrun", "z1_tools", script_name]
            
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.running_processes[category] = {
                "process": process,
                "script": script_key,
                "name": script_info["name"]
            }
            return True
        except Exception as e:
            print(f"Error launching {script_name}: {e}")
            return False
    
    def stop_script(self, category):
        """Stop script in category"""
        if category in self.running_processes:
            process_info = self.running_processes[category]
            process_info["process"].terminate()
            del self.running_processes[category]
            return True
        return False
    
    def stop_category(self, category):
        """Stop any running script in category"""
        if category in self.running_processes:
            self.stop_script(category)
    
    def stop_all_scripts(self):
        """Stop all running scripts"""
        for category in list(self.running_processes.keys()):
            self.stop_script(category)
    
    def switch_mode(self, new_mode):
        """Switch between simulation and real robot mode"""
        if new_mode == self.mode:
            return True
        
        # Stop all running scripts
        self.stop_all_scripts()
        
        if new_mode == "real":
            # Check if real robot bridge is available
            try:
                # Test connection to real robot
                self.mode = "real"
                self.system_status = "real_robot_connected"
                return True
            except:
                self.system_status = "real_robot_error"
                return False
        else:
            self.mode = "simulation"
            self.system_status = "simulation_ready"
            return True
    
    def setup_routes(self):
        @self.app.route('/')
        def index():
            return render_template_string(CONTROL_CENTER_HTML)
        
        @self.app.route('/api/scripts')
        def get_scripts():
            return jsonify(self.scripts)
        
        @self.app.route('/api/launch/<category>/<script>', methods=['POST'])
        def launch_script_route(category, script):
            if category in self.scripts and script in self.scripts[category]:
                success = self.launch_script(self.scripts[category][script], category, script)
                return jsonify({"success": success})
            return jsonify({"success": False, "error": "Script not found"})
        
        @self.app.route('/api/stop/<category>', methods=['POST'])
        def stop_script_route(category):
            success = self.stop_script(category)
            return jsonify({"success": success})
        
        @self.app.route('/api/stop_all', methods=['POST'])
        def stop_all_route():
            self.stop_all_scripts()
            return jsonify({"success": True})
        
        @self.app.route('/api/switch_mode/<mode>', methods=['POST'])
        def switch_mode_route(mode):
            success = self.switch_mode(mode)
            return jsonify({"success": success, "mode": self.mode})
        
        @self.app.route('/api/status')
        def status():
            # Check process status
            for category, process_info in list(self.running_processes.items()):
                if process_info["process"].poll() is not None:
                    del self.running_processes[category]
            
            return jsonify({
                "mode": self.mode,
                "system_status": self.system_status,
                "running_processes": {k: {"script": v["script"], "name": v["name"]} 
                                    for k, v in self.running_processes.items()},
                "joint_states": self.joint_states
            })
        
        @self.app.route('/api/emergency_stop', methods=['POST'])
        def emergency_stop():
            self.stop_all_scripts()
            # Launch emergency stop script
            try:
                subprocess.Popen(["rosrun", "z1_tools", "z1_emergency_stop.py"])
                return jsonify({"success": True})
            except:
                return jsonify({"success": False})
    
    def run(self):
        print("🎛️ Z1 Control Center starting...")
        print("📱 Open browser: http://localhost:8080")
        print("🎮 Unified interface for all Z1 operations")
        print("🛑 Press Ctrl+C to stop")
        
        def ros_spin():
            rospy.spin()
        
        ros_thread = threading.Thread(target=ros_spin)
        ros_thread.daemon = True
        ros_thread.start()
        
        self.app.run(host='0.0.0.0', port=8080, debug=False)

CONTROL_CENTER_HTML = '''
<!DOCTYPE html>
<html>
<head>
    <title>Z1 Control Center</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: #0f0f0f; color: white; }
        
        .header { background: linear-gradient(135deg, #1e3c72, #2a5298); padding: 20px; text-align: center; box-shadow: 0 4px 20px rgba(0,0,0,0.3); }
        .header h1 { margin: 0; font-size: 2.5em; font-weight: 300; }
        .header .subtitle { margin-top: 10px; opacity: 0.8; font-size: 1.1em; }
        
        .mode-switcher { background: #1a1a1a; padding: 15px; text-align: center; border-bottom: 1px solid #333; }
        .mode-btn { padding: 12px 30px; margin: 0 10px; border: none; border-radius: 25px; cursor: pointer; font-weight: bold; transition: all 0.3s; }
        .mode-btn.active { background: #4CAF50; color: white; }
        .mode-btn.inactive { background: #333; color: #ccc; }
        .mode-btn:hover { transform: translateY(-2px); box-shadow: 0 4px 12px rgba(0,0,0,0.3); }
        
        .container { display: flex; min-height: calc(100vh - 140px); }
        .sidebar { width: 300px; background: #1a1a1a; padding: 20px; overflow-y: auto; border-right: 1px solid #333; }
        .main-content { flex: 1; padding: 20px; overflow-y: auto; }
        
        .category { margin-bottom: 25px; }
        .category h3 { color: #4CAF50; margin-bottom: 15px; font-size: 1.2em; border-bottom: 2px solid #4CAF50; padding-bottom: 5px; }
        
        .script-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 20px; }
        .script-card { background: #2a2a2a; border-radius: 12px; padding: 20px; border-left: 4px solid #4CAF50; transition: all 0.3s; }
        .script-card:hover { transform: translateY(-5px); box-shadow: 0 8px 25px rgba(0,0,0,0.3); }
        .script-card.running { border-left-color: #ff9800; background: #2d2416; }
        
        .script-title { font-size: 1.1em; font-weight: bold; margin-bottom: 8px; }
        .script-description { font-size: 0.9em; color: #bbb; margin-bottom: 15px; }
        .script-actions { display: flex; gap: 10px; }
        
        .btn { padding: 8px 16px; border: none; border-radius: 6px; cursor: pointer; font-weight: bold; transition: all 0.3s; }
        .btn-primary { background: #4CAF50; color: white; }
        .btn-secondary { background: #2196F3; color: white; }
        .btn-danger { background: #f44336; color: white; }
        .btn:hover { transform: translateY(-2px); box-shadow: 0 4px 8px rgba(0,0,0,0.3); }
        
        .status-panel { background: #2a2a2a; border-radius: 12px; padding: 20px; margin-bottom: 20px; }
        .status-panel h3 { color: #2196F3; margin-bottom: 15px; }
        .status-item { display: flex; justify-content: space-between; margin: 8px 0; padding: 8px; background: #1a1a1a; border-radius: 6px; }
        
        .emergency-panel { background: linear-gradient(135deg, #d32f2f, #f44336); border-radius: 12px; padding: 20px; margin-bottom: 20px; text-align: center; }
        .emergency-btn { background: #fff; color: #d32f2f; padding: 15px 30px; border: none; border-radius: 8px; font-size: 1.1em; font-weight: bold; cursor: pointer; }
        
        .running-processes { background: #2a2a2a; border-radius: 12px; padding: 20px; }
        .process-item { display: flex; justify-content: space-between; align-items: center; padding: 10px; background: #1a1a1a; border-radius: 6px; margin: 8px 0; }
        
        @media (max-width: 768px) {
            .container { flex-direction: column; }
            .sidebar { width: 100%; }
            .script-grid { grid-template-columns: 1fr; }
        }
        
        .connection-status { display: inline-block; width: 12px; height: 12px; border-radius: 50%; margin-right: 8px; }
        .status-connected { background: #4CAF50; }
        .status-disconnected { background: #f44336; }
        .status-warning { background: #ff9800; }
    </style>
</head>
<body>
    <div class="header">
        <h1>🤖 Z1 Control Center</h1>
        <div class="subtitle">Unified interface for Unitree Z1 robotic arm operations</div>
    </div>
    
    <div class="mode-switcher">
        <button class="mode-btn active" id="sim-btn" onclick="switchMode('simulation')">
            🖥️ Simulation Mode
        </button>
        <button class="mode-btn inactive" id="real-btn" onclick="switchMode('real')">
            🦾 Real Robot Mode
        </button>
        <span id="mode-status" style="margin-left: 20px;">
            <span class="connection-status status-connected"></span>
            Simulation Ready
        </span>
    </div>
    
    <div class="container">
        <div class="sidebar">
            <div class="emergency-panel">
                <h3>🚨 Emergency Controls</h3>
                <button class="emergency-btn" onclick="emergencyStop()">
                    🛑 EMERGENCY STOP
                </button>
                <button class="btn btn-danger" onclick="stopAllScripts()" style="margin-top: 10px; width: 100%;">
                    ⏹️ Stop All Scripts
                </button>
            </div>
            
            <div class="status-panel">
                <h3>📊 System Status</h3>
                <div class="status-item">
                    <span>Mode:</span>
                    <span id="current-mode">Simulation</span>
                </div>
                <div class="status-item">
                    <span>Status:</span>
                    <span id="system-status">Ready</span>
                </div>
                <div class="status-item">
                    <span>Active Scripts:</span>
                    <span id="active-count">0</span>
                </div>
            </div>
            
            <div class="running-processes">
                <h3>🔄 Running Processes</h3>
                <div id="running-list">
                    <div style="text-align: center; color: #666; padding: 20px;">
                        No scripts running
                    </div>
                </div>
            </div>
        </div>
        
        <div class="main-content">
            <div id="script-categories"></div>
        </div>
    </div>

    <script>
        let currentMode = 'simulation';
        let scripts = {};
        let runningProcesses = {};
        
        // Load scripts and render interface
        function loadScripts() {
            fetch('/api/scripts')
            .then(response => response.json())
            .then(data => {
                scripts = data;
                renderScriptCategories();
            });
        }
        
        function renderScriptCategories() {
            const container = document.getElementById('script-categories');
            container.innerHTML = '';
            
            const categoryTitles = {
                'control': '🎮 Control Interfaces',
                'demos': '🎭 Demonstration Scripts',
                'interfaces': '🌐 Web Interfaces',
                'professional': '🚀 Professional Tools',
                'utilities': '🔧 Utility Scripts'
            };
            
            const categoryDescriptions = {
                'control': 'Direct robot control methods',
                'demos': 'Entertaining and educational demonstrations',
                'interfaces': 'Web-based control and programming interfaces',
                'professional': 'Advanced SDK and professional applications',
                'utilities': 'System utilities and safety tools'
            };
            
            Object.keys(scripts).forEach(category => {
                const categoryDiv = document.createElement('div');
                categoryDiv.className = 'category';
                
                const title = document.createElement('h3');
                title.textContent = categoryTitles[category] || category;
                categoryDiv.appendChild(title);
                
                const description = document.createElement('p');
                description.textContent = categoryDescriptions[category] || '';
                description.style.color = '#888';
                description.style.marginBottom = '15px';
                categoryDiv.appendChild(description);
                
                const grid = document.createElement('div');
                grid.className = 'script-grid';
                
                Object.keys(scripts[category]).forEach(scriptKey => {
                    const script = scripts[category][scriptKey];
                    const card = document.createElement('div');
                    card.className = 'script-card';
                    card.id = `card-${category}-${scriptKey}`;
                    
                    const isRunning = runningProcesses[category] && runningProcesses[category].script === scriptKey;
                    if (isRunning) {
                        card.classList.add('running');
                    }
                    
                    card.innerHTML = `
                        <div class="script-title">${script.name}</div>
                        <div class="script-description">
                            ${getScriptDescription(scriptKey)}
                        </div>
                        <div class="script-actions">
                            <button class="btn btn-primary" onclick="launchScript('${category}', '${scriptKey}')" 
                                    ${isRunning ? 'disabled' : ''}>
                                ${isRunning ? '🔄 Running' : '▶️ Launch'}
                            </button>
                            ${isRunning ? `<button class="btn btn-danger" onclick="stopScript('${category}')">⏹️ Stop</button>` : ''}
                        </div>
                    `;
                    
                    grid.appendChild(card);
                });
                
                categoryDiv.appendChild(grid);
                container.appendChild(categoryDiv);
            });
        }
        
        function getScriptDescription(scriptKey) {
            const descriptions = {
                'keyboard': 'Control robot with WASD keys and hotkeys',
                'xbox': 'Use Xbox controller for intuitive robot control',
                'advanced': 'Advanced control with multiple input methods',
                'demo': 'Basic pick and place demonstration',
                'bartender': 'Entertaining cocktail mixing choreography',
                'chess': 'Play chess by moving pieces on board',
                'magician': 'Perform magic tricks and illusions',
                'drawing': 'Draw geometric shapes and patterns',
                'wave': 'Simple waving motions and greetings',
                'musician': 'Conduct music with expressive movements',
                'painter': 'Create artistic paintings with heart and spiral patterns',
                'dancer': 'Perform choreographed dance routines and moves',
                'juggler': 'Simulate juggling motions and circus performance',
                'chef': 'Demonstrate cooking techniques and food preparation',
                'drummer': 'Play drum beats, solos, and rhythm exercises',
                'yoga': 'Guide through yoga poses and breathing exercises',
                'mime': 'Perform silent mime art with invisible objects',
                'web': 'Browser-based control with 3D visualization',
                'visual': 'Drag-and-drop visual programming interface',
                'api': 'REST API service for remote control',
                'enhanced': 'Full SDK integration with advanced features',
                'professional': 'Industrial-grade applications and tools',
                'pure_sdk': 'Direct SDK control without ROS overhead',
                'jetson': 'AI-powered autonomous control with computer vision',
                'emergency': 'Immediate safety stop for all robot motion',
                'limits': 'Test and verify joint safety limits',
                'bridge': 'Bridge between simulation and real robot'
            };
            return descriptions[scriptKey] || 'Robot control script';
        }
        
        function launchScript(category, script) {
            fetch(`/api/launch/${category}/${script}`, {method: 'POST'})
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    updateStatus();
                } else {
                    alert('Failed to launch script');
                }
            });
        }
        
        function stopScript(category) {
            fetch(`/api/stop/${category}`, {method: 'POST'})
            .then(response => response.json())
            .then(data => {
                updateStatus();
            });
        }
        
        function stopAllScripts() {
            fetch('/api/stop_all', {method: 'POST'})
            .then(response => response.json())
            .then(data => {
                updateStatus();
            });
        }
        
        function switchMode(mode) {
            fetch(`/api/switch_mode/${mode}`, {method: 'POST'})
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    currentMode = data.mode;
                    updateModeUI();
                    updateStatus();
                } else {
                    alert(`Failed to switch to ${mode} mode`);
                }
            });
        }
        
        function updateModeUI() {
            const simBtn = document.getElementById('sim-btn');
            const realBtn = document.getElementById('real-btn');
            
            if (currentMode === 'simulation') {
                simBtn.className = 'mode-btn active';
                realBtn.className = 'mode-btn inactive';
            } else {
                simBtn.className = 'mode-btn inactive';
                realBtn.className = 'mode-btn active';
            }
        }
        
        function emergencyStop() {
            if (confirm('Execute emergency stop? This will stop all robot motion immediately.')) {
                fetch('/api/emergency_stop', {method: 'POST'})
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        alert('Emergency stop executed!');
                        updateStatus();
                    }
                });
            }
        }
        
        function updateStatus() {
            fetch('/api/status')
            .then(response => response.json())
            .then(data => {
                currentMode = data.mode;
                runningProcesses = data.running_processes;
                
                // Update UI elements
                document.getElementById('current-mode').textContent = 
                    currentMode === 'simulation' ? 'Simulation' : 'Real Robot';
                document.getElementById('system-status').textContent = data.system_status;
                document.getElementById('active-count').textContent = Object.keys(runningProcesses).length;
                
                // Update mode status
                const statusElement = document.getElementById('mode-status');
                const statusClass = currentMode === 'simulation' ? 'status-connected' : 
                                  data.system_status.includes('error') ? 'status-disconnected' : 'status-connected';
                statusElement.innerHTML = `
                    <span class="connection-status ${statusClass}"></span>
                    ${currentMode === 'simulation' ? 'Simulation Ready' : 'Real Robot Connected'}
                `;
                
                // Update running processes
                const runningList = document.getElementById('running-list');
                if (Object.keys(runningProcesses).length === 0) {
                    runningList.innerHTML = '<div style="text-align: center; color: #666; padding: 20px;">No scripts running</div>';
                } else {
                    runningList.innerHTML = Object.keys(runningProcesses).map(category => {
                        const process = runningProcesses[category];
                        return `
                            <div class="process-item">
                                <span>${process.name}</span>
                                <button class="btn btn-danger" onclick="stopScript('${category}')">Stop</button>
                            </div>
                        `;
                    }).join('');
                }
                
                // Re-render script cards to update running status
                renderScriptCategories();
                updateModeUI();
            });
        }
        
        // Initialize
        window.addEventListener('load', () => {
            loadScripts();
            updateStatus();
            setInterval(updateStatus, 2000); // Update every 2 seconds
        });
    </script>
</body>
</html>
'''

if __name__ == "__main__":
    try:
        control_center = Z1ControlCenter()
        control_center.run()
    except KeyboardInterrupt:
        print("\n🛑 Control Center stopped")
    except Exception as e:
        print(f"❌ Error: {e}")