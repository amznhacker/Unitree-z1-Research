#!/usr/bin/env python3

"""
Z1 Visual Programmer - Scratch-like programming for Z1 robot
Access at: http://localhost:8081
"""

import rospy
import json
import time
import threading
from flask import Flask, render_template_string, request, jsonify
from unitree_legged_msgs.msg import MotorCmd
from sensor_msgs.msg import JointState

class Z1VisualProgrammer:
    def __init__(self):
        rospy.init_node("z1_visual_programmer")
        
        # Joint limits and publishers
        self.limits = {
            "Joint01": (-1.2, 1.2), "Joint02": (-1.0, 1.0), "Joint03": (0.0, 2.4),
            "Joint04": (-1.2, 1.2), "Joint05": (-1.0, 1.0), "Joint06": (-1.2, 1.2),
            "Gripper": (0.0, 0.6)
        }
        
        self.positions = {j: 0.0 for j in self.limits.keys()}
        self.pubs = {}
        
        for joint in self.limits.keys():
            controller = f"{joint}_controller" if joint != "Gripper" else "Gripper_controller"
            topic = f"/z1_gazebo/{controller}/command"
            self.pubs[joint] = rospy.Publisher(topic, MotorCmd, queue_size=1)
        
        # Program execution
        self.program = []
        self.running = False
        self.execution_thread = None
        
        # Flask app
        self.app = Flask(__name__)
        self.setup_routes()
    
    def move_joint(self, joint, position, duration=1.0):
        """Move joint to position over duration"""
        start_pos = self.positions[joint]
        target_pos = max(self.limits[joint][0], min(self.limits[joint][1], float(position)))
        
        steps = int(duration * 50)  # 50Hz
        for i in range(steps + 1):
            if not self.running:
                break
            
            progress = i / steps
            current_pos = start_pos + (target_pos - start_pos) * progress
            
            msg = MotorCmd()
            msg.mode = 10
            msg.q = current_pos
            msg.Kp = 35.0
            msg.Kd = 1.5
            
            self.pubs[joint].publish(msg)
            self.positions[joint] = current_pos
            time.sleep(0.02)  # 50Hz
    
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
        
        self.running = False
    
    def setup_routes(self):
        @self.app.route('/')
        def index():
            return render_template_string(VISUAL_PROGRAMMER_HTML)
        
        @self.app.route('/api/run', methods=['POST'])
        def run_program():
            if self.running:
                return jsonify({"success": False, "error": "Program already running"})
            
            program = request.json.get('program', [])
            self.execution_thread = threading.Thread(target=self.execute_program, args=(program,))
            self.execution_thread.start()
            
            return jsonify({"success": True})
        
        @self.app.route('/api/stop', methods=['POST'])
        def stop_program():
            self.running = False
            return jsonify({"success": True})
        
        @self.app.route('/api/status')
        def status():
            return jsonify({
                "running": self.running,
                "positions": self.positions
            })
    
    def run(self):
        print("🎨 Z1 Visual Programmer starting...")
        print("📱 Open browser: http://localhost:8081")
        print("🛑 Press Ctrl+C to stop")
        
        def ros_spin():
            rospy.spin()
        
        ros_thread = threading.Thread(target=ros_spin)
        ros_thread.daemon = True
        ros_thread.start()
        
        self.app.run(host='0.0.0.0', port=8081, debug=False)

VISUAL_PROGRAMMER_HTML = '''
<!DOCTYPE html>
<html>
<head>
    <title>Z1 Visual Programmer</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; margin: 0; background: #f0f0f0; }
        .container { display: flex; height: 100vh; }
        .palette { width: 250px; background: #2c3e50; color: white; padding: 20px; overflow-y: auto; }
        .workspace { flex: 1; background: white; padding: 20px; overflow-y: auto; }
        .controls { width: 200px; background: #34495e; color: white; padding: 20px; }
        
        .block { margin: 10px 0; padding: 15px; border-radius: 8px; cursor: pointer; user-select: none; }
        .block:hover { opacity: 0.8; }
        
        .move-block { background: #3498db; }
        .wait-block { background: #e74c3c; }
        .preset-block { background: #2ecc71; }
        .repeat-block { background: #9b59b6; }
        
        .program-block { margin: 5px 0; padding: 10px; border-radius: 5px; position: relative; }
        .program-block .remove { position: absolute; top: 5px; right: 5px; background: #e74c3c; color: white; border: none; border-radius: 3px; cursor: pointer; }
        
        .input-group { margin: 5px 0; }
        .input-group label { display: block; font-size: 12px; margin-bottom: 3px; }
        .input-group select, .input-group input { width: 100%; padding: 3px; border: none; border-radius: 3px; }
        
        .btn { padding: 10px 20px; margin: 5px; border: none; border-radius: 5px; cursor: pointer; font-weight: bold; }
        .btn-run { background: #27ae60; color: white; }
        .btn-stop { background: #e74c3c; color: white; }
        .btn-clear { background: #95a5a6; color: white; }
        
        .status { margin: 10px 0; padding: 10px; background: #ecf0f1; border-radius: 5px; font-family: monospace; font-size: 12px; }
        
        h3 { margin-top: 0; }
    </style>
</head>
<body>
    <div class="container">
        <!-- Block Palette -->
        <div class="palette">
            <h3>🧩 Blocks</h3>
            
            <div class="block move-block" onclick="addBlock('move')">
                📍 Move Joint
                <div style="font-size: 12px; margin-top: 5px;">Move a joint to position</div>
            </div>
            
            <div class="block wait-block" onclick="addBlock('wait')">
                ⏱️ Wait
                <div style="font-size: 12px; margin-top: 5px;">Pause for time</div>
            </div>
            
            <div class="block preset-block" onclick="addBlock('preset')">
                🎯 Go to Preset
                <div style="font-size: 12px; margin-top: 5px;">Move to preset position</div>
            </div>
            
            <div class="block repeat-block" onclick="addBlock('repeat')">
                🔄 Repeat
                <div style="font-size: 12px; margin-top: 5px;">Repeat actions</div>
            </div>
        </div>
        
        <!-- Workspace -->
        <div class="workspace">
            <h2>🎨 Visual Program</h2>
            <div id="program" style="min-height: 400px; border: 2px dashed #bdc3c7; border-radius: 10px; padding: 20px;">
                <div style="text-align: center; color: #7f8c8d; margin-top: 100px;">
                    Drag blocks here to create your program
                </div>
            </div>
        </div>
        
        <!-- Controls -->
        <div class="controls">
            <h3>🎮 Controls</h3>
            
            <button class="btn btn-run" onclick="runProgram()">▶️ Run</button>
            <button class="btn btn-stop" onclick="stopProgram()">⏹️ Stop</button>
            <button class="btn btn-clear" onclick="clearProgram()">🗑️ Clear</button>
            
            <div class="status">
                <h4>Status</h4>
                <div id="status">Ready</div>
            </div>
            
            <div class="status">
                <h4>Examples</h4>
                <button class="btn" onclick="loadExample('wave')" style="width: 100%; margin: 2px 0;">👋 Wave</button>
                <button class="btn" onclick="loadExample('dance')" style="width: 100%; margin: 2px 0;">💃 Dance</button>
                <button class="btn" onclick="loadExample('pickup')" style="width: 100%; margin: 2px 0;">📦 Pick Up</button>
            </div>
        </div>
    </div>

    <script>
        let program = [];
        let blockId = 0;
        
        function addBlock(type) {
            const block = { id: ++blockId, type: type };
            
            if (type === 'move') {
                block.joint = 'Joint01';
                block.position = 0;
                block.duration = 1;
            } else if (type === 'wait') {
                block.duration = 1;
            } else if (type === 'preset') {
                block.preset = 'home';
                block.duration = 2;
            } else if (type === 'repeat') {
                block.times = 2;
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
                container.innerHTML = '<div style="text-align: center; color: #7f8c8d; margin-top: 100px;">Drag blocks here to create your program</div>';
                return;
            }
            
            container.innerHTML = program.map(block => {
                let content = '';
                
                if (block.type === 'move') {
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
                } else if (block.type === 'wait') {
                    content = `
                        <div class="input-group">
                            <label>Duration (s):</label>
                            <input type="number" step="0.1" value="${block.duration}" 
                                   onchange="updateBlock(${block.id}, 'duration', this.value)">
                        </div>
                    `;
                } else if (block.type === 'preset') {
                    content = `
                        <div class="input-group">
                            <label>Preset:</label>
                            <select onchange="updateBlock(${block.id}, 'preset', this.value)">
                                ${['home', 'ready', 'wave'].map(p => 
                                    `<option value="${p}" ${block.preset === p ? 'selected' : ''}>${p}</option>`
                                ).join('')}
                            </select>
                        </div>
                        <div class="input-group">
                            <label>Duration (s):</label>
                            <input type="number" step="0.1" value="${block.duration}" 
                                   onchange="updateBlock(${block.id}, 'duration', this.value)">
                        </div>
                    `;
                }
                
                const blockClass = block.type + '-block';
                const title = {
                    'move': '📍 Move Joint',
                    'wait': '⏱️ Wait',
                    'preset': '🎯 Go to Preset',
                    'repeat': '🔄 Repeat'
                }[block.type];
                
                return `
                    <div class="program-block ${blockClass}">
                        <button class="remove" onclick="removeBlock(${block.id})">×</button>
                        <strong>${title}</strong>
                        ${content}
                    </div>
                `;
            }).join('');
        }
        
        function runProgram() {
            fetch('/api/run', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({program: program})
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    document.getElementById('status').textContent = 'Running...';
                } else {
                    alert('Error: ' + data.error);
                }
            });
        }
        
        function stopProgram() {
            fetch('/api/stop', {method: 'POST'})
            .then(response => response.json())
            .then(data => {
                document.getElementById('status').textContent = 'Stopped';
            });
        }
        
        function clearProgram() {
            program = [];
            renderProgram();
        }
        
        function loadExample(example) {
            if (example === 'wave') {
                program = [
                    {id: ++blockId, type: 'preset', preset: 'ready', duration: 2},
                    {id: ++blockId, type: 'move', joint: 'Joint01', position: 0.5, duration: 1},
                    {id: ++blockId, type: 'move', joint: 'Joint06', position: 0.5, duration: 0.5},
                    {id: ++blockId, type: 'move', joint: 'Joint06', position: -0.5, duration: 0.5},
                    {id: ++blockId, type: 'move', joint: 'Joint06', position: 0.5, duration: 0.5},
                    {id: ++blockId, type: 'move', joint: 'Joint06', position: 0, duration: 0.5},
                    {id: ++blockId, type: 'preset', preset: 'home', duration: 2}
                ];
            } else if (example === 'dance') {
                program = [
                    {id: ++blockId, type: 'preset', preset: 'ready', duration: 1},
                    {id: ++blockId, type: 'move', joint: 'Joint01', position: 0.8, duration: 1},
                    {id: ++blockId, type: 'move', joint: 'Joint01', position: -0.8, duration: 1},
                    {id: ++blockId, type: 'move', joint: 'Joint01', position: 0, duration: 1},
                    {id: ++blockId, type: 'preset', preset: 'home', duration: 2}
                ];
            } else if (example === 'pickup') {
                program = [
                    {id: ++blockId, type: 'preset', preset: 'ready', duration: 2},
                    {id: ++blockId, type: 'move', joint: 'Gripper', position: 0.6, duration: 1},
                    {id: ++blockId, type: 'move', joint: 'Joint03', position: 1.5, duration: 2},
                    {id: ++blockId, type: 'move', joint: 'Gripper', position: 0, duration: 1},
                    {id: ++blockId, type: 'wait', duration: 1},
                    {id: ++blockId, type: 'preset', preset: 'ready', duration: 2},
                    {id: ++blockId, type: 'preset', preset: 'home', duration: 2}
                ];
            }
            renderProgram();
        }
        
        // Update status periodically
        setInterval(() => {
            fetch('/api/status')
            .then(response => response.json())
            .then(data => {
                if (!data.running && document.getElementById('status').textContent === 'Running...') {
                    document.getElementById('status').textContent = 'Ready';
                }
            });
        }, 1000);
        
        // Initialize
        renderProgram();
    </script>
</body>
</html>
'''

if __name__ == "__main__":
    try:
        programmer = Z1VisualProgrammer()
        programmer.run()
    except KeyboardInterrupt:
        print("\n🛑 Visual Programmer stopped")
    except Exception as e:
        print(f"❌ Error: {e}")