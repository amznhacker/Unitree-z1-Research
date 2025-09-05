#!/usr/bin/env python3

"""
Z1 Enhanced Web GUI - Advanced web control with 3D visualization
Access at: http://localhost:8080
Features: Real-time 3D robot visualization, mobile-friendly interface, advanced controls
"""

import rospy
import json
import math
from flask import Flask, render_template_string, request, jsonify
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import threading
import time

class Z1EnhancedWebGUI:
    def __init__(self):
        rospy.init_node("z1_enhanced_web_gui")
        
        # Joint limits
        self.limits = {
            "Joint01": (-1.2, 1.2), "Joint02": (-1.0, 1.0), "Joint03": (0.0, 2.4),
            "Joint04": (-1.2, 1.2), "Joint05": (-1.0, 1.0), "Joint06": (-1.2, 1.2),
            "Gripper": (0.0, 0.6)
        }
        
        # Current positions and states
        self.positions = {j: 0.0 for j in self.limits.keys()}
        self.joint_states = {}
        self.target_positions = {j: 0.0 for j in self.limits.keys()}
        
        # Publishers
        self.pubs = {}
        for joint in self.limits.keys():
            controller = f"{joint}_controller" if joint != "Gripper" else "Gripper_controller"
            topic = f"/z1_gazebo/{controller}/command"
            self.pubs[joint] = rospy.Publisher(topic, Float64, queue_size=1)
        
        # Subscriber for joint states
        rospy.Subscriber("/z1_gazebo/joint_states", JointState, self.joint_state_callback)
        
        # Trajectory execution
        self.executing_trajectory = False
        self.trajectory_thread = None
        
        # Flask app
        self.app = Flask(__name__)
        self.setup_routes()
        
    def joint_state_callback(self, msg):
        """Update current joint positions from robot"""
        if len(msg.position) >= 6:
            self.joint_states = {
                "Joint01": msg.position[0], "Joint02": msg.position[1], "Joint03": msg.position[2],
                "Joint04": msg.position[3], "Joint05": msg.position[4], "Joint06": msg.position[5],
                "Gripper": msg.position[6] if len(msg.position) > 6 else 0.0
            }
    
    def move_joint(self, joint, position):
        """Move joint to position"""
        min_pos, max_pos = self.limits[joint]
        position = max(min_pos, min(max_pos, float(position)))
        
        msg = Float64()
        msg.data = position
        
        self.pubs[joint].publish(msg)
        self.positions[joint] = position
        return position
    
    def execute_trajectory(self, trajectory):
        """Execute smooth trajectory"""
        self.executing_trajectory = True
        
        for waypoint in trajectory:
            if not self.executing_trajectory:
                break
            
            # Move to waypoint
            for joint, position in waypoint['positions'].items():
                if joint in self.pubs:
                    self.move_joint(joint, position)
            
            # Wait for duration
            time.sleep(waypoint.get('duration', 1.0))
        
        self.executing_trajectory = False
    
    def calculate_forward_kinematics(self, joint_angles):
        """Simple forward kinematics calculation"""
        # Simplified FK for demonstration
        j1, j2, j3 = joint_angles.get('Joint01', 0), joint_angles.get('Joint02', 0), joint_angles.get('Joint03', 0)
        
        # Link lengths (approximate)
        l1, l2, l3 = 0.3, 0.3, 0.2
        
        # Calculate end-effector position
        x = (l2 * math.cos(j2) + l3 * math.cos(j2 + j3)) * math.cos(j1)
        y = (l2 * math.cos(j2) + l3 * math.cos(j2 + j3)) * math.sin(j1)
        z = l1 + l2 * math.sin(j2) + l3 * math.sin(j2 + j3)
        
        return {"x": x, "y": y, "z": z}
    
    def setup_routes(self):
        @self.app.route('/')
        def index():
            return render_template_string(ENHANCED_HTML_TEMPLATE)
        
        @self.app.route('/api/move', methods=['POST'])
        def move():
            data = request.json
            joint = data.get('joint')
            position = data.get('position')
            
            if joint in self.limits:
                actual_pos = self.move_joint(joint, position)
                return jsonify({"success": True, "position": actual_pos})
            return jsonify({"success": False, "error": "Invalid joint"})
        
        @self.app.route('/api/move_multiple', methods=['POST'])
        def move_multiple():
            data = request.json
            positions = data.get('positions', {})
            duration = data.get('duration', 0)
            
            results = {}
            for joint, position in positions.items():
                if joint in self.limits:
                    results[joint] = self.move_joint(joint, position)
            
            if duration > 0:
                time.sleep(duration)
            
            return jsonify({"success": True, "positions": results})
        
        @self.app.route('/api/trajectory', methods=['POST'])
        def execute_trajectory_route():
            if self.executing_trajectory:
                return jsonify({"success": False, "error": "Trajectory already executing"})
            
            trajectory = request.json.get('trajectory', [])
            self.trajectory_thread = threading.Thread(target=self.execute_trajectory, args=(trajectory,))
            self.trajectory_thread.start()
            
            return jsonify({"success": True})
        
        @self.app.route('/api/stop_trajectory', methods=['POST'])
        def stop_trajectory():
            self.executing_trajectory = False
            return jsonify({"success": True})
        
        @self.app.route('/api/status')
        def status():
            fk_result = self.calculate_forward_kinematics(self.joint_states)
            return jsonify({
                "positions": self.positions,
                "joint_states": self.joint_states,
                "limits": self.limits,
                "executing_trajectory": self.executing_trajectory,
                "end_effector": fk_result
            })
        
        @self.app.route('/api/preset/<preset>')
        def preset(preset):
            presets = {
                "home": {j: 0.0 for j in self.limits.keys()},
                "ready": {"Joint01": 0.0, "Joint02": -0.5, "Joint03": 1.0, "Joint04": 0.0, "Joint05": -0.5, "Joint06": 0.0, "Gripper": 0.0},
                "wave": {"Joint01": 0.5, "Joint02": -0.3, "Joint03": 1.2, "Joint04": 0.0, "Joint05": -0.9, "Joint06": 0.0, "Gripper": 0.3},
                "pick": {"Joint01": 0.0, "Joint02": -0.8, "Joint03": 1.5, "Joint04": 0.0, "Joint05": -0.7, "Joint06": 0.0, "Gripper": 0.6},
                "place": {"Joint01": 0.8, "Joint02": -0.5, "Joint03": 1.0, "Joint04": 0.0, "Joint05": -0.5, "Joint06": 0.0, "Gripper": 0.0}
            }
            
            if preset in presets:
                for joint, pos in presets[preset].items():
                    self.move_joint(joint, pos)
                return jsonify({"success": True})
            return jsonify({"success": False, "error": "Unknown preset"})
        
        @self.app.route('/api/emergency_stop', methods=['POST'])
        def emergency_stop():
            """Emergency stop - freeze all joints"""
            self.executing_trajectory = False
            # Keep current positions
            for joint in self.limits.keys():
                current_pos = self.joint_states.get(joint, 0.0)
                self.move_joint(joint, current_pos)
            return jsonify({"success": True})
    
    def run(self):
        """Start web server"""
        print("🌐 Z1 Enhanced Web GUI starting...")
        print("📱 Open browser: http://localhost:8080")
        print("✨ Features: 3D visualization, mobile-friendly, advanced controls")
        print("🛑 Press Ctrl+C to stop")
        
        # Start ROS spinner in background
        def ros_spin():
            rospy.spin()
        
        ros_thread = threading.Thread(target=ros_spin)
        ros_thread.daemon = True
        ros_thread.start()
        
        # Start Flask server
        self.app.run(host='0.0.0.0', port=8080, debug=False)

ENHANCED_HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>Z1 Enhanced Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: #1a1a1a; color: white; }
        
        .container { display: flex; height: 100vh; }
        .sidebar { width: 350px; background: #2d2d2d; padding: 20px; overflow-y: auto; }
        .main-view { flex: 1; display: flex; flex-direction: column; }
        .viewer { flex: 1; position: relative; background: #000; }
        .controls-bar { height: 60px; background: #333; display: flex; align-items: center; padding: 0 20px; gap: 10px; }
        
        .joint-control { margin: 15px 0; padding: 15px; background: #3a3a3a; border-radius: 8px; }
        .joint-control h4 { margin-bottom: 10px; color: #4CAF50; }
        .slider { width: 100%; height: 6px; border-radius: 3px; background: #555; outline: none; }
        .slider::-webkit-slider-thumb { appearance: none; width: 20px; height: 20px; border-radius: 50%; background: #4CAF50; cursor: pointer; }
        
        .btn { padding: 8px 16px; border: none; border-radius: 6px; cursor: pointer; font-weight: bold; transition: all 0.3s; }
        .btn-primary { background: #4CAF50; color: white; }
        .btn-secondary { background: #2196F3; color: white; }
        .btn-danger { background: #f44336; color: white; }
        .btn-warning { background: #ff9800; color: white; }
        .btn:hover { transform: translateY(-2px); box-shadow: 0 4px 8px rgba(0,0,0,0.3); }
        
        .preset-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin: 15px 0; }
        .status-panel { background: #2a2a2a; padding: 15px; border-radius: 8px; margin: 15px 0; }
        .status-panel h4 { color: #2196F3; margin-bottom: 10px; }
        .status-text { font-family: 'Courier New', monospace; font-size: 12px; line-height: 1.4; }
        
        .trajectory-panel { background: #2a2a2a; padding: 15px; border-radius: 8px; margin: 15px 0; }
        .trajectory-panel h4 { color: #ff9800; margin-bottom: 10px; }
        
        .mobile-controls { display: none; }
        
        @media (max-width: 768px) {
            .container { flex-direction: column; }
            .sidebar { width: 100%; height: 40vh; }
            .main-view { height: 60vh; }
            .mobile-controls { display: block; }
            .joint-control { margin: 8px 0; padding: 10px; }
        }
        
        .loading { position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); }
        .spinner { border: 4px solid #333; border-top: 4px solid #4CAF50; border-radius: 50%; width: 40px; height: 40px; animation: spin 1s linear infinite; }
        @keyframes spin { 0% { transform: rotate(0deg); } 100% { transform: rotate(360deg); } }
        
        .end-effector-info { background: #1a4d1a; padding: 10px; border-radius: 6px; margin: 10px 0; }
    </style>
</head>
<body>
    <div class="container">
        <!-- Sidebar Controls -->
        <div class="sidebar">
            <h2>🤖 Z1 Enhanced Control</h2>
            
            <!-- Quick Presets -->
            <div class="status-panel">
                <h4>Quick Presets</h4>
                <div class="preset-grid">
                    <button class="btn btn-primary" onclick="setPreset('home')">🏠 Home</button>
                    <button class="btn btn-primary" onclick="setPreset('ready')">⚡ Ready</button>
                    <button class="btn btn-primary" onclick="setPreset('wave')">👋 Wave</button>
                    <button class="btn btn-primary" onclick="setPreset('pick')">📦 Pick</button>
                </div>
            </div>
            
            <!-- Joint Controls -->
            <div id="joint-controls"></div>
            
            <!-- End Effector Position -->
            <div class="end-effector-info">
                <h4>🎯 End Effector Position</h4>
                <div id="end-effector-pos">X: 0.00, Y: 0.00, Z: 0.00</div>
            </div>
            
            <!-- Trajectory Controls -->
            <div class="trajectory-panel">
                <h4>🎬 Trajectory Control</h4>
                <button class="btn btn-secondary" onclick="executePickPlace()" style="width: 100%; margin: 5px 0;">📦 Pick & Place</button>
                <button class="btn btn-secondary" onclick="executeWaveSequence()" style="width: 100%; margin: 5px 0;">👋 Wave Sequence</button>
                <button class="btn btn-danger" onclick="stopTrajectory()" style="width: 100%; margin: 5px 0;">⏹️ Stop</button>
            </div>
            
            <!-- Status -->
            <div class="status-panel">
                <h4>📊 Robot Status</h4>
                <div class="status-text" id="status">Connecting...</div>
            </div>
        </div>
        
        <!-- Main View -->
        <div class="main-view">
            <!-- Controls Bar -->
            <div class="controls-bar">
                <button class="btn btn-danger" onclick="emergencyStop()">🛑 EMERGENCY STOP</button>
                <button class="btn btn-warning" onclick="toggleView()">🔄 Toggle View</button>
                <span style="margin-left: auto; font-size: 14px;" id="connection-status">🔴 Disconnected</span>
            </div>
            
            <!-- 3D Viewer -->
            <div class="viewer" id="viewer">
                <div class="loading" id="loading">
                    <div class="spinner"></div>
                    <div style="margin-top: 10px;">Loading 3D Viewer...</div>
                </div>
            </div>
        </div>
    </div>

    <script>
        // Global variables
        let scene, camera, renderer, robot, controls;
        let joints = ['Joint01', 'Joint02', 'Joint03', 'Joint04', 'Joint05', 'Joint06', 'Gripper'];
        let jointNames = ['Base', 'Shoulder', 'Elbow', 'Forearm', 'Wrist Pitch', 'Wrist Roll', 'Gripper'];
        let currentPositions = {};
        let isConnected = false;
        
        // Initialize 3D scene
        function init3D() {
            const viewer = document.getElementById('viewer');
            
            // Scene setup
            scene = new THREE.Scene();
            scene.background = new THREE.Color(0x222222);
            
            // Camera setup
            camera = new THREE.PerspectiveCamera(75, viewer.clientWidth / viewer.clientHeight, 0.1, 1000);
            camera.position.set(2, 2, 2);
            
            // Renderer setup
            renderer = new THREE.WebGLRenderer({ antialias: true });
            renderer.setSize(viewer.clientWidth, viewer.clientHeight);
            renderer.shadowMap.enabled = true;
            renderer.shadowMap.type = THREE.PCFSoftShadowMap;
            viewer.appendChild(renderer.domElement);
            
            // Controls
            controls = new THREE.OrbitControls(camera, renderer.domElement);
            controls.enableDamping = true;
            controls.dampingFactor = 0.05;
            
            // Lighting
            const ambientLight = new THREE.AmbientLight(0x404040, 0.6);
            scene.add(ambientLight);
            
            const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
            directionalLight.position.set(5, 5, 5);
            directionalLight.castShadow = true;
            scene.add(directionalLight);
            
            // Create robot visualization
            createRobotVisualization();
            
            // Ground plane
            const groundGeometry = new THREE.PlaneGeometry(4, 4);
            const groundMaterial = new THREE.MeshLambertMaterial({ color: 0x333333 });
            const ground = new THREE.Mesh(groundGeometry, groundMaterial);
            ground.rotation.x = -Math.PI / 2;
            ground.receiveShadow = true;
            scene.add(ground);
            
            // Grid
            const gridHelper = new THREE.GridHelper(4, 20, 0x444444, 0x444444);
            scene.add(gridHelper);
            
            // Hide loading
            document.getElementById('loading').style.display = 'none';
            
            // Start render loop
            animate();
        }
        
        function createRobotVisualization() {
            robot = new THREE.Group();
            
            // Base
            const baseGeometry = new THREE.CylinderGeometry(0.1, 0.15, 0.2, 16);
            const baseMaterial = new THREE.MeshLambertMaterial({ color: 0x4CAF50 });
            const base = new THREE.Mesh(baseGeometry, baseMaterial);
            base.position.y = 0.1;
            base.castShadow = true;
            robot.add(base);
            
            // Links (simplified representation)
            const linkMaterial = new THREE.MeshLambertMaterial({ color: 0x2196F3 });
            
            // Link 1
            const link1Geometry = new THREE.BoxGeometry(0.05, 0.3, 0.05);
            const link1 = new THREE.Mesh(link1Geometry, linkMaterial);
            link1.position.set(0, 0.35, 0);
            link1.castShadow = true;
            robot.add(link1);
            
            // Link 2
            const link2Geometry = new THREE.BoxGeometry(0.3, 0.05, 0.05);
            const link2 = new THREE.Mesh(link2Geometry, linkMaterial);
            link2.position.set(0.15, 0.5, 0);
            link2.castShadow = true;
            robot.add(link2);
            
            // Link 3
            const link3Geometry = new THREE.BoxGeometry(0.25, 0.05, 0.05);
            const link3 = new THREE.Mesh(link3Geometry, linkMaterial);
            link3.position.set(0.425, 0.5, 0);
            link3.castShadow = true;
            robot.add(link3);
            
            // End effector
            const eeGeometry = new THREE.SphereGeometry(0.03, 8, 8);
            const eeMaterial = new THREE.MeshLambertMaterial({ color: 0xff9800 });
            const endEffector = new THREE.Mesh(eeGeometry, eeMaterial);
            endEffector.position.set(0.55, 0.5, 0);
            endEffector.castShadow = true;
            robot.add(endEffector);
            
            scene.add(robot);
        }
        
        function animate() {
            requestAnimationFrame(animate);
            controls.update();
            
            // Update robot visualization based on joint positions
            if (robot && currentPositions.Joint01 !== undefined) {
                robot.rotation.y = currentPositions.Joint01 || 0;
            }
            
            renderer.render(scene, camera);
        }
        
        // Handle window resize
        window.addEventListener('resize', () => {
            const viewer = document.getElementById('viewer');
            camera.aspect = viewer.clientWidth / viewer.clientHeight;
            camera.updateProjectionMatrix();
            renderer.setSize(viewer.clientWidth, viewer.clientHeight);
        });
        
        // Create joint controls
        function createJointControls() {
            const container = document.getElementById('joint-controls');
            container.innerHTML = '';
            
            joints.forEach((joint, i) => {
                const div = document.createElement('div');
                div.className = 'joint-control';
                div.innerHTML = `
                    <h4>${jointNames[i]} (${joint})</h4>
                    <input type="range" class="slider" id="${joint}" 
                           min="-1.2" max="1.2" step="0.01" value="0"
                           oninput="moveJoint('${joint}', this.value)">
                    <div style="display: flex; justify-content: space-between; margin-top: 5px;">
                        <span style="font-size: 12px;">-1.2</span>
                        <span id="${joint}_value" style="font-weight: bold;">0.00</span>
                        <span style="font-size: 12px;">1.2</span>
                    </div>
                `;
                container.appendChild(div);
            });
        }
        
        // Move joint
        function moveJoint(joint, position) {
            document.getElementById(joint + '_value').textContent = parseFloat(position).toFixed(2);
            currentPositions[joint] = parseFloat(position);
            
            fetch('/api/move', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({joint: joint, position: parseFloat(position)})
            });
        }
        
        // Set preset
        function setPreset(preset) {
            fetch(`/api/preset/${preset}`)
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        updateStatus();
                    }
                });
        }
        
        // Execute pick and place trajectory
        function executePickPlace() {
            const trajectory = [
                { positions: {Joint01: 0, Joint02: -0.5, Joint03: 1.0, Joint04: 0, Joint05: -0.5, Joint06: 0, Gripper: 0.6}, duration: 2 },
                { positions: {Joint01: 0, Joint02: -0.8, Joint03: 1.5, Joint04: 0, Joint05: -0.7, Joint06: 0, Gripper: 0.6}, duration: 2 },
                { positions: {Joint01: 0, Joint02: -0.8, Joint03: 1.5, Joint04: 0, Joint05: -0.7, Joint06: 0, Gripper: 0.0}, duration: 1 },
                { positions: {Joint01: 0.8, Joint02: -0.5, Joint03: 1.0, Joint04: 0, Joint05: -0.5, Joint06: 0, Gripper: 0.0}, duration: 2 },
                { positions: {Joint01: 0.8, Joint02: -0.8, Joint03: 1.5, Joint04: 0, Joint05: -0.7, Joint06: 0, Gripper: 0.0}, duration: 2 },
                { positions: {Joint01: 0.8, Joint02: -0.8, Joint03: 1.5, Joint04: 0, Joint05: -0.7, Joint06: 0, Gripper: 0.6}, duration: 1 },
                { positions: {Joint01: 0, Joint02: 0, Joint03: 0, Joint04: 0, Joint05: 0, Joint06: 0, Gripper: 0.6}, duration: 3 }
            ];
            
            fetch('/api/trajectory', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({trajectory: trajectory})
            });
        }
        
        // Execute wave sequence
        function executeWaveSequence() {
            const trajectory = [
                { positions: {Joint01: 0, Joint02: -0.3, Joint03: 1.2, Joint04: 0, Joint05: -0.9, Joint06: 0, Gripper: 0.3}, duration: 2 },
                { positions: {Joint01: 0, Joint02: -0.3, Joint03: 1.2, Joint04: 0, Joint05: -0.9, Joint06: 0.5, Gripper: 0.3}, duration: 0.5 },
                { positions: {Joint01: 0, Joint02: -0.3, Joint03: 1.2, Joint04: 0, Joint05: -0.9, Joint06: -0.5, Gripper: 0.3}, duration: 0.5 },
                { positions: {Joint01: 0, Joint02: -0.3, Joint03: 1.2, Joint04: 0, Joint05: -0.9, Joint06: 0.5, Gripper: 0.3}, duration: 0.5 },
                { positions: {Joint01: 0, Joint02: -0.3, Joint03: 1.2, Joint04: 0, Joint05: -0.9, Joint06: 0, Gripper: 0.3}, duration: 0.5 },
                { positions: {Joint01: 0, Joint02: 0, Joint03: 0, Joint04: 0, Joint05: 0, Joint06: 0, Gripper: 0}, duration: 2 }
            ];
            
            fetch('/api/trajectory', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({trajectory: trajectory})
            });
        }
        
        // Stop trajectory
        function stopTrajectory() {
            fetch('/api/stop_trajectory', {method: 'POST'});
        }
        
        // Emergency stop
        function emergencyStop() {
            fetch('/api/emergency_stop', {method: 'POST'})
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        alert('🛑 Emergency stop activated!');
                    }
                });
        }
        
        // Toggle view
        function toggleView() {
            if (camera.position.z > 0) {
                camera.position.set(-2, 2, -2);
            } else {
                camera.position.set(2, 2, 2);
            }
        }
        
        // Update status
        function updateStatus() {
            fetch('/api/status')
                .then(response => response.json())
                .then(data => {
                    isConnected = true;
                    document.getElementById('connection-status').innerHTML = '🟢 Connected';
                    
                    // Update sliders
                    Object.keys(data.positions).forEach(joint => {
                        const slider = document.getElementById(joint);
                        const value = document.getElementById(joint + '_value');
                        if (slider && value) {
                            slider.value = data.positions[joint];
                            value.textContent = data.positions[joint].toFixed(2);
                            currentPositions[joint] = data.positions[joint];
                        }
                    });
                    
                    // Update end effector position
                    if (data.end_effector) {
                        const ee = data.end_effector;
                        document.getElementById('end-effector-pos').textContent = 
                            `X: ${ee.x.toFixed(3)}, Y: ${ee.y.toFixed(3)}, Z: ${ee.z.toFixed(3)}`;
                    }
                    
                    // Update status display
                    let statusText = 'Joint States:\\n';
                    Object.keys(data.joint_states).forEach(joint => {
                        statusText += `${joint}: ${data.joint_states[joint].toFixed(2)}°\\n`;
                    });
                    
                    if (data.executing_trajectory) {
                        statusText += '\\n🎬 Executing trajectory...';
                    }
                    
                    document.getElementById('status').textContent = statusText;
                })
                .catch(error => {
                    isConnected = false;
                    document.getElementById('connection-status').innerHTML = '🔴 Disconnected';
                });
        }
        
        // Initialize everything
        window.addEventListener('load', () => {
            init3D();
            createJointControls();
            updateStatus();
            setInterval(updateStatus, 1000); // Update every second
        });
    </script>
</body>
</html>
'''

if __name__ == "__main__":
    try:
        gui = Z1EnhancedWebGUI()
        gui.run()
    except KeyboardInterrupt:
        print("\n🛑 Enhanced Web GUI stopped")
    except Exception as e:
        print(f"❌ Error: {e}")