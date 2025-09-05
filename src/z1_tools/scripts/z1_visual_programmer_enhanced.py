#!/usr/bin/env python3

"""
Z1 Enhanced Visual Programmer - Advanced drag-and-drop programming with 3D preview
Access at: http://localhost:8081
Features: 3D robot visualization, sequence recording, advanced block types, simulation
"""

import rospy
import json
import time
import threading
import math
from flask import Flask, render_template_string, request, jsonify
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class Z1EnhancedVisualProgrammer:
    def __init__(self):
        rospy.init_node("z1_enhanced_visual_programmer")
        
        # Joint limits and publishers
        self.limits = {
            "Joint01": (-1.2, 1.2), "Joint02": (-1.0, 1.0), "Joint03": (0.0, 2.4),
            "Joint04": (-1.2, 1.2), "Joint05": (-1.0, 1.0), "Joint06": (-1.2, 1.2),
            "Gripper": (0.0, 0.6)
        }
        
        self.positions = {j: 0.0 for j in self.limits.keys()}
        self.joint_states = {}
        self.pubs = {}
        
        for joint in self.limits.keys():
            controller = f"{joint}_controller" if joint != "Gripper" else "Gripper_controller"
            topic = f"/z1_gazebo/{controller}/command"
            self.pubs[joint] = rospy.Publisher(topic, Float64, queue_size=1)
        
        # Subscribe to joint states for real-time feedback
        rospy.Subscriber("/z1_gazebo/joint_states", JointState, self.joint_state_callback)
        
        # Program execution and recording
        self.program = []
        self.running = False
        self.execution_thread = None
        self.recording = False
        self.recorded_sequence = []
        self.last_record_time = 0
        
        # Simulation mode
        self.simulation_mode = False
        self.simulated_positions = {j: 0.0 for j in self.limits.keys()}
        
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
            
            # Record positions if recording
            if self.recording:
                current_time = time.time()
                if current_time - self.last_record_time > 0.1:  # Record every 100ms
                    self.recorded_sequence.append({
                        'type': 'recorded_position',
                        'positions': self.joint_states.copy(),
                        'timestamp': current_time
                    })
                    self.last_record_time = current_time
    
    def move_joint(self, joint, position, duration=1.0, simulate_only=False):
        """Move joint to position over duration"""
        start_pos = self.simulated_positions[joint] if simulate_only else self.positions[joint]
        target_pos = max(self.limits[joint][0], min(self.limits[joint][1], float(position)))
        
        steps = int(duration * 50)  # 50Hz
        for i in range(steps + 1):
            if not self.running and not simulate_only:
                break
            
            progress = i / steps
            current_pos = start_pos + (target_pos - start_pos) * progress
            
            if simulate_only:
                self.simulated_positions[joint] = current_pos
            else:
                msg = Float64()
                msg.data = current_pos
                self.pubs[joint].publish(msg)
                self.positions[joint] = current_pos
            
            if not simulate_only:
                time.sleep(0.02)  # 50Hz
    
    def calculate_forward_kinematics(self, joint_angles):
        """Calculate end-effector position from joint angles"""
        j1 = joint_angles.get('Joint01', 0)
        j2 = joint_angles.get('Joint02', 0)
        j3 = joint_angles.get('Joint03', 0)
        j4 = joint_angles.get('Joint04', 0)
        j5 = joint_angles.get('Joint05', 0)
        
        # Simplified FK calculation (DH parameters would be more accurate)
        l1, l2, l3 = 0.3, 0.3, 0.2  # Link lengths
        
        x = (l2 * math.cos(j2) + l3 * math.cos(j2 + j3)) * math.cos(j1)
        y = (l2 * math.cos(j2) + l3 * math.cos(j2 + j3)) * math.sin(j1)
        z = l1 + l2 * math.sin(j2) + l3 * math.sin(j2 + j3)
        
        return {"x": x, "y": y, "z": z}
    
    def simple_inverse_kinematics(self, x, y, z):
        """Simple IK solver for positioning"""
        # Base rotation
        j1 = math.atan2(y, x)
        
        # Planar distance
        r = math.sqrt(x*x + y*y)
        
        # Height adjustment
        z_adj = z - 0.3  # Subtract base height
        
        # 2-link planar IK
        l2, l3 = 0.3, 0.2
        d = math.sqrt(r*r + z_adj*z_adj)
        
        if d > (l2 + l3):
            return None  # Out of reach
        
        # Elbow up solution
        cos_j3 = (d*d - l2*l2 - l3*l3) / (2 * l2 * l3)
        if abs(cos_j3) > 1:
            return None
        
        j3 = math.acos(cos_j3)
        j2 = math.atan2(z_adj, r) - math.atan2(l3 * math.sin(j3), l2 + l3 * math.cos(j3))
        
        return {
            "Joint01": j1,
            "Joint02": j2,
            "Joint03": j3,
            "Joint04": 0.0,
            "Joint05": -(j2 + j3),  # Keep end-effector level
            "Joint06": 0.0
        }
    
    def simulate_program(self, program):
        """Simulate program execution without moving real robot"""
        self.simulated_positions = {j: 0.0 for j in self.limits.keys()}
        simulation_path = []
        
        for block in program:
            block_type = block.get('type')
            
            if block_type == 'move':
                joint = block.get('joint')
                position = float(block.get('position', 0))
                duration = float(block.get('duration', 1))
                
                # Simulate movement
                start_pos = self.simulated_positions[joint]
                target_pos = max(self.limits[joint][0], min(self.limits[joint][1], position))
                
                steps = max(10, int(duration * 10))  # 10Hz for simulation
                for i in range(steps + 1):
                    progress = i / steps
                    current_pos = start_pos + (target_pos - start_pos) * progress
                    self.simulated_positions[joint] = current_pos
                    
                    # Calculate FK for this configuration
                    fk_result = self.calculate_forward_kinematics(self.simulated_positions)
                    simulation_path.append({
                        'positions': self.simulated_positions.copy(),
                        'end_effector': fk_result,
                        'time': i * duration / steps
                    })
            
            elif block_type == 'ik':
                x = float(block.get('x', 0))
                y = float(block.get('y', 0))
                z = float(block.get('z', 0))
                duration = float(block.get('duration', 2))
                
                ik_result = self.simple_inverse_kinematics(x, y, z)
                if ik_result:
                    for joint, position in ik_result.items():
                        self.simulated_positions[joint] = position
                    
                    fk_result = self.calculate_forward_kinematics(self.simulated_positions)
                    simulation_path.append({
                        'positions': self.simulated_positions.copy(),
                        'end_effector': fk_result,
                        'time': duration
                    })
        
        return simulation_path
    
    def execute_program(self, program):
        """Execute visual program"""
        self.running = True
        
        for block in program:
            if not self.running:
                break
                
            block_type = block.get('type')
            
            if block_type == 'move':
                joint = block.get('joint')
                position = float(block.get('position', 0))
                duration = float(block.get('duration', 1))
                self.move_joint(joint, position, duration)
                
            elif block_type == 'wait':
                duration = float(block.get('duration', 1))
                time.sleep(duration)
                
            elif block_type == 'preset':
                preset_name = block.get('preset')
                presets = {
                    "home": {j: 0.0 for j in self.limits.keys()},
                    "ready": {"Joint01": 0.0, "Joint02": -0.5, "Joint03": 1.0, "Joint04": 0.0, "Joint05": -0.5, "Joint06": 0.0, "Gripper": 0.0},
                    "wave": {"Joint01": 0.5, "Joint02": -0.3, "Joint03": 1.2, "Joint04": 0.0, "Joint05": -0.9, "Joint06": 0.0, "Gripper": 0.3}
                }
                if preset_name in presets:
                    duration = float(block.get('duration', 2))
                    for joint, pos in presets[preset_name].items():
                        threading.Thread(target=self.move_joint, args=(joint, pos, duration)).start()
                    time.sleep(duration)
            
            elif block_type == 'ik':
                x = float(block.get('x', 0))
                y = float(block.get('y', 0))
                z = float(block.get('z', 0))
                duration = float(block.get('duration', 2))
                
                ik_result = self.simple_inverse_kinematics(x, y, z)
                if ik_result:
                    for joint, pos in ik_result.items():
                        threading.Thread(target=self.move_joint, args=(joint, pos, duration)).start()
                    time.sleep(duration)
            
            elif block_type == 'repeat':
                times = int(block.get('times', 2))
                sub_program = block.get('sub_program', [])
                for _ in range(times):
                    if not self.running:
                        break
                    self.execute_program(sub_program)
        
        self.running = False
    
    def setup_routes(self):
        @self.app.route('/')
        def index():
            return render_template_string(ENHANCED_VISUAL_PROGRAMMER_HTML)
        
        @self.app.route('/api/run', methods=['POST'])
        def run_program():
            if self.running:
                return jsonify({"success": False, "error": "Program already running"})
            
            program = request.json.get('program', [])
            self.execution_thread = threading.Thread(target=self.execute_program, args=(program,))
            self.execution_thread.start()
            
            return jsonify({"success": True})
        
        @self.app.route('/api/simulate', methods=['POST'])
        def simulate():
            program = request.json.get('program', [])
            simulation_path = self.simulate_program(program)
            return jsonify({"success": True, "path": simulation_path})
        
        @self.app.route('/api/stop', methods=['POST'])
        def stop_program():
            self.running = False
            return jsonify({"success": True})
        
        @self.app.route('/api/status')
        def status():
            fk_result = self.calculate_forward_kinematics(self.joint_states)
            return jsonify({
                "running": self.running,
                "positions": self.positions,
                "joint_states": self.joint_states,
                "recording": self.recording,
                "recorded_frames": len(self.recorded_sequence),
                "end_effector": fk_result
            })
        
        @self.app.route('/api/record/start', methods=['POST'])
        def start_record():
            self.recording = True
            self.recorded_sequence = []
            self.last_record_time = time.time()
            return jsonify({"success": True, "recording": self.recording})
        
        @self.app.route('/api/record/stop', methods=['POST'])
        def stop_record():
            self.recording = False
            return jsonify({"success": True, "recording": self.recording, "frames": len(self.recorded_sequence)})
        
        @self.app.route('/api/inverse_kinematics', methods=['POST'])
        def inverse_kinematics():
            data = request.json
            x = float(data.get('x', 0))
            y = float(data.get('y', 0))
            z = float(data.get('z', 0))
            
            ik_result = self.simple_inverse_kinematics(x, y, z)
            if ik_result:
                return jsonify({"success": True, "joints": ik_result})
            else:
                return jsonify({"success": False, "error": "Position out of reach"})
    
    def run(self):
        print("🎨 Z1 Enhanced Visual Programmer starting...")
        print("📱 Open browser: http://localhost:8081")
        print("✨ Features: 3D visualization, simulation, advanced blocks, recording")
        print("🛑 Press Ctrl+C to stop")
        
        def ros_spin():
            rospy.spin()
        
        ros_thread = threading.Thread(target=ros_spin)
        ros_thread.daemon = True
        ros_thread.start()
        
        self.app.run(host='0.0.0.0', port=8081, debug=False)

ENHANCED_VISUAL_PROGRAMMER_HTML = '''
<!DOCTYPE html>
<html>
<head>
    <title>Z1 Enhanced Visual Programmer</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: #1a1a1a; color: white; }
        
        .container { display: flex; height: 100vh; }
        .palette { width: 280px; background: #2c3e50; color: white; padding: 15px; overflow-y: auto; }
        .workspace { flex: 1; display: flex; flex-direction: column; }
        .workspace-header { height: 60px; background: #34495e; display: flex; align-items: center; padding: 0 20px; gap: 10px; }
        .workspace-content { flex: 1; display: flex; }
        .program-area { flex: 1; background: #2a2a2a; padding: 20px; overflow-y: auto; }
        .preview-area { width: 400px; background: #1a1a1a; display: flex; flex-direction: column; }
        .controls { width: 250px; background: #34495e; color: white; padding: 15px; overflow-y: auto; }
        
        .block { margin: 8px 0; padding: 12px; border-radius: 8px; cursor: pointer; user-select: none; transition: all 0.3s; }
        .block:hover { transform: translateY(-2px); box-shadow: 0 4px 12px rgba(0,0,0,0.3); }
        
        .move-block { background: linear-gradient(135deg, #3498db, #2980b9); }
        .wait-block { background: linear-gradient(135deg, #e74c3c, #c0392b); }
        .preset-block { background: linear-gradient(135deg, #2ecc71, #27ae60); }
        .repeat-block { background: linear-gradient(135deg, #9b59b6, #8e44ad); }
        .ik-block { background: linear-gradient(135deg, #f39c12, #e67e22); }
        .record-block { background: linear-gradient(135deg, #8e44ad, #7d3c98); }
        .sensor-block { background: linear-gradient(135deg, #1abc9c, #16a085); }
        .logic-block { background: linear-gradient(135deg, #34495e, #2c3e50); }
        
        .program-block { margin: 8px 0; padding: 12px; border-radius: 8px; position: relative; border-left: 4px solid #3498db; }
        .program-block .remove { position: absolute; top: 8px; right: 8px; background: #e74c3c; color: white; border: none; border-radius: 50%; width: 24px; height: 24px; cursor: pointer; font-size: 12px; }
        
        .input-group { margin: 8px 0; }
        .input-group label { display: block; font-size: 12px; margin-bottom: 4px; color: #bdc3c7; }
        .input-group select, .input-group input { width: 100%; padding: 6px; border: 1px solid #555; border-radius: 4px; background: #3a3a3a; color: white; }
        
        .btn { padding: 10px 20px; margin: 5px; border: none; border-radius: 6px; cursor: pointer; font-weight: bold; transition: all 0.3s; }
        .btn:hover { transform: translateY(-2px); box-shadow: 0 4px 8px rgba(0,0,0,0.3); }
        .btn-run { background: linear-gradient(135deg, #27ae60, #2ecc71); color: white; }
        .btn-simulate { background: linear-gradient(135deg, #3498db, #2980b9); color: white; }
        .btn-stop { background: linear-gradient(135deg, #e74c3c, #c0392b); color: white; }
        .btn-clear { background: linear-gradient(135deg, #95a5a6, #7f8c8d); color: white; }
        
        .status { margin: 10px 0; padding: 12px; background: #2a2a2a; border-radius: 6px; font-family: 'Courier New', monospace; font-size: 12px; }
        .preview-canvas { flex: 1; background: #000; }
        
        h3 { margin: 10px 0; color: #3498db; }
        h4 { margin: 8px 0; color: #2ecc71; }
        
        .drag-area { min-height: 400px; border: 2px dashed #555; border-radius: 10px; padding: 20px; background: #2a2a2a; }
        .drag-area.drag-over { border-color: #3498db; background: #1e3a5f; }
        
        .simulation-controls { padding: 15px; background: #2a2a2a; border-bottom: 1px solid #555; }
        .simulation-path { height: 200px; overflow-y: auto; padding: 10px; background: #1a1a1a; font-family: monospace; font-size: 11px; }
        
        @media (max-width: 1200px) {
            .preview-area { width: 300px; }
            .controls { width: 200px; }
        }
        
        @media (max-width: 768px) {
            .container { flex-direction: column; }
            .palette { width: 100%; height: 200px; }
            .preview-area { width: 100%; height: 300px; }
        }
    </style>
</head>
<body>
    <div class="container">
        <!-- Block Palette -->
        <div class="palette">
            <h3>🧩 Advanced Blocks</h3>
            
            <div class="block move-block" onclick="addBlock('move')">
                📍 Move Joint
                <div style="font-size: 11px; margin-top: 4px; opacity: 0.8;">Move specific joint to position</div>
            </div>
            
            <div class="block wait-block" onclick="addBlock('wait')">
                ⏱️ Wait
                <div style="font-size: 11px; margin-top: 4px; opacity: 0.8;">Pause execution for time</div>
            </div>
            
            <div class="block preset-block" onclick="addBlock('preset')">
                🎯 Go to Preset
                <div style="font-size: 11px; margin-top: 4px; opacity: 0.8;">Move to predefined position</div>
            </div>
            
            <div class="block ik-block" onclick="addBlock('ik')">
                🎯 Move to XYZ
                <div style="font-size: 11px; margin-top: 4px; opacity: 0.8;">Position end-effector in space</div>
            </div>
            
            <div class="block repeat-block" onclick="addBlock('repeat')">
                🔄 Repeat
                <div style="font-size: 11px; margin-top: 4px; opacity: 0.8;">Repeat sequence of actions</div>
            </div>
            
            <div class="block record-block" onclick="addBlock('record')">
                📹 Play Recording
                <div style="font-size: 11px; margin-top: 4px; opacity: 0.8;">Replay recorded motion</div>
            </div>
            
            <div class="block sensor-block" onclick="addBlock('sensor')">
                📡 Wait for Sensor
                <div style="font-size: 11px; margin-top: 4px; opacity: 0.8;">Wait for sensor condition</div>
            </div>
            
            <div class="block logic-block" onclick="addBlock('if')">
                🔀 If Condition
                <div style="font-size: 11px; margin-top: 4px; opacity: 0.8;">Conditional execution</div>
            </div>
        </div>
        
        <!-- Workspace -->
        <div class="workspace">
            <!-- Header -->
            <div class="workspace-header">
                <h2>🎨 Enhanced Visual Program</h2>
                <div style="margin-left: auto; display: flex; gap: 10px;">
                    <button class="btn btn-simulate" onclick="simulateProgram()">🔍 Simulate</button>
                    <button class="btn btn-run" onclick="runProgram()">▶️ Run</button>
                    <button class="btn btn-stop" onclick="stopProgram()">⏹️ Stop</button>
                    <button class="btn btn-clear" onclick="clearProgram()">🗑️ Clear</button>
                </div>
            </div>
            
            <!-- Content -->
            <div class="workspace-content">
                <!-- Program Area -->
                <div class="program-area">
                    <div id="program" class="drag-area">
                        <div style="text-align: center; color: #7f8c8d; margin-top: 100px;">
                            <h3>Drag blocks here to create your program</h3>
                            <p>Use advanced blocks for complex robot behaviors</p>
                        </div>
                    </div>
                </div>
                
                <!-- Preview Area -->
                <div class="preview-area">
                    <div class="simulation-controls">
                        <h4>🔍 3D Preview & Simulation</h4>
                        <button class="btn" onclick="resetView()" style="width: 100%; margin: 5px 0; font-size: 12px;">🔄 Reset View</button>
                    </div>
                    <div class="preview-canvas" id="preview-canvas"></div>
                    <div class="simulation-path" id="simulation-path">
                        Simulation results will appear here...
                    </div>
                </div>
            </div>
        </div>
        
        <!-- Controls -->
        <div class="controls">
            <h3>🎮 Controls</h3>
            
            <div class="status">
                <h4>Status</h4>
                <div id="status">Ready</div>
            </div>
            
            <div class="status">
                <h4>Recording</h4>
                <button class="btn" onclick="toggleRecording()" id="recordBtn" style="width: 100%; margin: 2px 0; background: #e74c3c;">🔴 Record</button>
                <div id="recordStatus">No recording</div>
            </div>
            
            <div class="status">
                <h4>End Effector</h4>
                <div id="endEffectorPos">X: 0.00, Y: 0.00, Z: 0.00</div>
            </div>
            
            <div class="status">
                <h4>Examples</h4>
                <button class="btn" onclick="loadExample('wave')" style="width: 100%; margin: 2px 0; font-size: 12px;">👋 Wave</button>
                <button class="btn" onclick="loadExample('pickup')" style="width: 100%; margin: 2px 0; font-size: 12px;">📦 Pick Up</button>
                <button class="btn" onclick="loadExample('dance')" style="width: 100%; margin: 2px 0; font-size: 12px;">💃 Dance</button>
                <button class="btn" onclick="loadExample('advanced')" style="width: 100%; margin: 2px 0; font-size: 12px;">🚀 Advanced</button>
            </div>
        </div>
    </div>

    <script>
        // Global variables
        let program = [];
        let blockId = 0;
        let recording = false;
        let scene, camera, renderer, robot, controls;
        let simulationPath = [];
        
        // Initialize 3D preview
        function init3DPreview() {
            const canvas = document.getElementById('preview-canvas');
            
            scene = new THREE.Scene();
            scene.background = new THREE.Color(0x111111);
            
            camera = new THREE.PerspectiveCamera(60, canvas.clientWidth / canvas.clientHeight, 0.1, 1000);
            camera.position.set(1.5, 1.5, 1.5);
            
            renderer = new THREE.WebGLRenderer({ antialias: true });
            renderer.setSize(canvas.clientWidth, canvas.clientHeight);
            renderer.shadowMap.enabled = true;
            canvas.appendChild(renderer.domElement);
            
            controls = new THREE.OrbitControls(camera, renderer.domElement);
            controls.enableDamping = true;
            
            // Lighting
            const ambientLight = new THREE.AmbientLight(0x404040, 0.4);
            scene.add(ambientLight);
            
            const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
            directionalLight.position.set(2, 2, 2);
            directionalLight.castShadow = true;
            scene.add(directionalLight);
            
            // Create simple robot visualization
            createRobotVisualization();
            
            // Ground
            const groundGeometry = new THREE.PlaneGeometry(2, 2);
            const groundMaterial = new THREE.MeshLambertMaterial({ color: 0x333333 });
            const ground = new THREE.Mesh(groundGeometry, groundMaterial);
            ground.rotation.x = -Math.PI / 2;
            scene.add(ground);
            
            animate3D();
        }
        
        function createRobotVisualization() {
            robot = new THREE.Group();
            
            // Base
            const baseGeometry = new THREE.CylinderGeometry(0.08, 0.12, 0.15, 12);
            const baseMaterial = new THREE.MeshLambertMaterial({ color: 0x4CAF50 });
            const base = new THREE.Mesh(baseGeometry, baseMaterial);
            base.position.y = 0.075;
            robot.add(base);
            
            // Links
            const linkMaterial = new THREE.MeshLambertMaterial({ color: 0x2196F3 });
            
            // Link 1 (vertical)
            const link1Geometry = new THREE.BoxGeometry(0.04, 0.25, 0.04);
            const link1 = new THREE.Mesh(link1Geometry, linkMaterial);
            link1.position.set(0, 0.275, 0);
            robot.add(link1);
            
            // Link 2 (horizontal)
            const link2Geometry = new THREE.BoxGeometry(0.25, 0.04, 0.04);
            const link2 = new THREE.Mesh(link2Geometry, linkMaterial);
            link2.position.set(0.125, 0.4, 0);
            robot.add(link2);
            
            // Link 3
            const link3Geometry = new THREE.BoxGeometry(0.2, 0.04, 0.04);
            const link3 = new THREE.Mesh(link3Geometry, linkMaterial);
            link3.position.set(0.35, 0.4, 0);
            robot.add(link3);
            
            // End effector
            const eeGeometry = new THREE.SphereGeometry(0.025, 8, 8);
            const eeMaterial = new THREE.MeshLambertMaterial({ color: 0xff9800 });
            const endEffector = new THREE.Mesh(eeGeometry, eeMaterial);
            endEffector.position.set(0.45, 0.4, 0);
            robot.add(endEffector);
            
            scene.add(robot);
        }
        
        function animate3D() {
            requestAnimationFrame(animate3D);
            controls.update();
            renderer.render(scene, camera);
        }
        
        function resetView() {
            camera.position.set(1.5, 1.5, 1.5);
            controls.reset();
        }
        
        // Block management
        function addBlock(type) {
            const block = { id: ++blockId, type: type };
            
            // Set default values based on block type
            switch(type) {
                case 'move':
                    block.joint = 'Joint01';
                    block.position = 0;
                    block.duration = 1;
                    break;
                case 'wait':
                    block.duration = 1;
                    break;
                case 'preset':
                    block.preset = 'home';
                    block.duration = 2;
                    break;
                case 'ik':
                    block.x = 0.3;
                    block.y = 0.0;
                    block.z = 0.2;
                    block.duration = 2;
                    break;
                case 'repeat':
                    block.times = 2;
                    block.sub_program = [];
                    break;
                case 'record':
                    block.action = 'play';
                    break;
                case 'sensor':
                    block.sensor = 'force';
                    block.threshold = 10;
                    break;
                case 'if':
                    block.condition = 'joint_position';
                    block.joint = 'Joint01';
                    block.operator = '>';
                    block.value = 0;
                    block.then_program = [];
                    block.else_program = [];
                    break;
            }
            
            program.push(block);
            renderProgram();
        }
        
        function removeBlock(id) {
            program = program.filter(b => b.id !== id);
            renderProgram();
        }
        
        function updateBlock(id, field, value) {
            const block = program.find(b => b.id === id);
            if (block) {
                block[field] = value;
            }
        }
        
        function renderProgram() {
            const container = document.getElementById('program');
            
            if (program.length === 0) {
                container.innerHTML = `
                    <div style="text-align: center; color: #7f8c8d; margin-top: 100px;">
                        <h3>Drag blocks here to create your program</h3>
                        <p>Use advanced blocks for complex robot behaviors</p>
                    </div>
                `;
                return;
            }
            
            container.innerHTML = program.map(block => {
                let content = '';
                let blockClass = block.type + '-block';
                
                // Generate content based on block type
                switch(block.type) {
                    case 'move':
                        content = `
                            <div class="input-group">
                                <label>Joint:</label>
                                <select onchange="updateBlock(${block.id}, 'joint', this.value)">
                                    ${['Joint01', 'Joint02', 'Joint03', 'Joint04', 'Joint05', 'Joint06', 'Gripper'].map(j => 
                                        `<option value="${j}" ${block.joint === j ? 'selected' : ''}>${j}</option>`
                                    ).join('')}
                                </select>
                            </div>
                            <div class="input-group">
                                <label>Position:</label>
                                <input type="number" step="0.1" value="${block.position}" 
                                       onchange="updateBlock(${block.id}, 'position', this.value)">
                            </div>
                            <div class="input-group">
                                <label>Duration (s):</label>
                                <input type="number" step="0.1" value="${block.duration}" 
                                       onchange="updateBlock(${block.id}, 'duration', this.value)">
                            </div>
                        `;
                        break;
                    case 'ik':
                        content = `
                            <div class="input-group">
                                <label>X Position:</label>
                                <input type="number" step="0.01" value="${block.x}" 
                                       onchange="updateBlock(${block.id}, 'x', this.value)">
                            </div>
                            <div class="input-group">
                                <label>Y Position:</label>
                                <input type="number" step="0.01" value="${block.y}" 
                                       onchange="updateBlock(${block.id}, 'y', this.value)">
                            </div>
                            <div class="input-group">
                                <label>Z Position:</label>
                                <input type="number" step="0.01" value="${block.z}" 
                                       onchange="updateBlock(${block.id}, 'z', this.value)">
                            </div>
                            <div class="input-group">
                                <label>Duration (s):</label>
                                <input type="number" step="0.1" value="${block.duration}" 
                                       onchange="updateBlock(${block.id}, 'duration', this.value)">
                            </div>
                        `;
                        break;
                    // Add other block types...
                }
                
                const titles = {
                    'move': '📍 Move Joint',
                    'wait': '⏱️ Wait',
                    'preset': '🎯 Go to Preset',
                    'ik': '🎯 Move to XYZ',
                    'repeat': '🔄 Repeat',
                    'record': '📹 Play Recording',
                    'sensor': '📡 Wait for Sensor',
                    'if': '🔀 If Condition'
                };
                
                return `
                    <div class="program-block ${blockClass}">
                        <button class="remove" onclick="removeBlock(${block.id})">×</button>
                        <strong>${titles[block.type] || block.type}</strong>
                        ${content}
                    </div>
                `;
            }).join('');
        }
        
        // Program execution
        function runProgram() {
            fetch('/api/run', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({program: program})
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    document.getElementById('status').textContent = 'Running program...';
                } else {
                    alert('Error: ' + data.error);
                }
            });
        }
        
        function simulateProgram() {
            fetch('/api/simulate', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({program: program})
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    simulationPath = data.path;
                    displaySimulationPath(data.path);
                    document.getElementById('status').textContent = 'Simulation complete';
                }
            });
        }
        
        function displaySimulationPath(path) {
            const pathDiv = document.getElementById('simulation-path');
            let pathText = 'Simulation Path:\\n\\n';
            
            path.forEach((point, i) => {
                pathText += `Step ${i + 1}:\\n`;
                pathText += `  End Effector: X=${point.end_effector.x.toFixed(3)}, Y=${point.end_effector.y.toFixed(3)}, Z=${point.end_effector.z.toFixed(3)}\\n`;
                pathText += `  Time: ${point.time.toFixed(2)}s\\n\\n`;
            });
            
            pathDiv.textContent = pathText;
        }
        
        function stopProgram() {
            fetch('/api/stop', {method: 'POST'});
        }
        
        function clearProgram() {
            program = [];
            renderProgram();
        }
        
        function toggleRecording() {
            if (recording) {
                fetch('/api/record/stop', {method: 'POST'})
                .then(response => response.json())
                .then(data => {
                    recording = false;
                    document.getElementById('recordBtn').innerHTML = '🔴 Record';
                    document.getElementById('recordStatus').textContent = `Recorded ${data.frames} frames`;
                });
            } else {
                fetch('/api/record/start', {method: 'POST'})
                .then(response => response.json())
                .then(data => {
                    recording = true;
                    document.getElementById('recordBtn').innerHTML = '⏹️ Stop';
                    document.getElementById('recordStatus').textContent = 'Recording...';
                });
            }
        }
        
        function loadExample(example) {
            switch(example) {
                case 'wave':
                    program = [
                        {id: ++blockId, type: 'preset', preset: 'ready', duration: 2},
                        {id: ++blockId, type: 'move', joint: 'Joint06', position: 0.5, duration: 0.5},
                        {id: ++blockId, type: 'move', joint: 'Joint06', position: -0.5, duration: 0.5},
                        {id: ++blockId, type: 'move', joint: 'Joint06', position: 0.5, duration: 0.5},
                        {id: ++blockId, type: 'move', joint: 'Joint06', position: 0, duration: 0.5},
                        {id: ++blockId, type: 'preset', preset: 'home', duration: 2}
                    ];
                    break;
                case 'pickup':
                    program = [
                        {id: ++blockId, type: 'preset', preset: 'ready', duration: 2},
                        {id: ++blockId, type: 'ik', x: 0.3, y: 0.0, z: 0.1, duration: 2},
                        {id: ++blockId, type: 'move', joint: 'Gripper', position: 0.0, duration: 1},
                        {id: ++blockId, type: 'ik', x: 0.3, y: 0.0, z: 0.3, duration: 2},
                        {id: ++blockId, type: 'preset', preset: 'home', duration: 2}
                    ];
                    break;
                case 'advanced':
                    program = [
                        {id: ++blockId, type: 'preset', preset: 'ready', duration: 1},
                        {id: ++blockId, type: 'repeat', times: 3, sub_program: [
                            {id: ++blockId, type: 'ik', x: 0.2, y: 0.2, z: 0.2, duration: 1},
                            {id: ++blockId, type: 'ik', x: 0.2, y: -0.2, z: 0.2, duration: 1}
                        ]},
                        {id: ++blockId, type: 'preset', preset: 'home', duration: 2}
                    ];
                    break;
            }
            renderProgram();
        }
        
        // Update status periodically
        function updateStatus() {
            fetch('/api/status')
            .then(response => response.json())
            .then(data => {
                if (!data.running && document.getElementById('status').textContent === 'Running program...') {
                    document.getElementById('status').textContent = 'Ready';
                }
                
                if (data.end_effector) {
                    const ee = data.end_effector;
                    document.getElementById('endEffectorPos').textContent = 
                        `X: ${ee.x.toFixed(3)}, Y: ${ee.y.toFixed(3)}, Z: ${ee.z.toFixed(3)}`;
                }
            });
        }
        
        // Initialize everything
        window.addEventListener('load', () => {
            init3DPreview();
            renderProgram();
            updateStatus();
            setInterval(updateStatus, 1000);
        });
        
        // Handle window resize
        window.addEventListener('resize', () => {
            const canvas = document.getElementById('preview-canvas');
            camera.aspect = canvas.clientWidth / canvas.clientHeight;
            camera.updateProjectionMatrix();
            renderer.setSize(canvas.clientWidth, canvas.clientHeight);
        });
    </script>
</body>
</html>
'''

if __name__ == "__main__":
    try:
        programmer = Z1EnhancedVisualProgrammer()
        programmer.run()
    except KeyboardInterrupt:
        print("\n🛑 Enhanced Visual Programmer stopped")
    except Exception as e:
        print(f"❌ Error: {e}")