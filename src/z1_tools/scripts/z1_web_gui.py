#!/usr/bin/env python3

"""
Z1 Web GUI - Control Unitree Z1 via web browser
Access at: http://localhost:8080
"""

import rospy
import json
from flask import Flask, render_template_string, request, jsonify
from unitree_legged_msgs.msg import MotorCmd
from sensor_msgs.msg import JointState
import threading
import time

class Z1WebGUI:
    def __init__(self):
        rospy.init_node("z1_web_gui")
        
        # Joint limits
        self.limits = {
            "Joint01": (-1.2, 1.2), "Joint02": (-1.0, 1.0), "Joint03": (0.0, 2.4),
            "Joint04": (-1.2, 1.2), "Joint05": (-1.0, 1.0), "Joint06": (-1.2, 1.2),
            "Gripper": (0.0, 0.6)
        }
        
        # Current positions
        self.positions = {j: 0.0 for j in self.limits.keys()}
        self.joint_states = {}
        
        # Publishers
        self.pubs = {}
        for joint in self.limits.keys():
            controller = f"{joint}_controller" if joint != "Gripper" else "Gripper_controller"
            topic = f"/z1_gazebo/{controller}/command"
            self.pubs[joint] = rospy.Publisher(topic, MotorCmd, queue_size=1)
        
        # Subscriber for joint states
        rospy.Subscriber("/z1_gazebo/joint_states", JointState, self.joint_state_callback)
        
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
        
        msg = MotorCmd()
        msg.mode = 10
        msg.q = position
        msg.Kp = 35.0
        msg.Kd = 1.5
        
        self.pubs[joint].publish(msg)
        self.positions[joint] = position
        return position
    
    def setup_routes(self):
        @self.app.route('/')
        def index():
            return render_template_string(HTML_TEMPLATE)
        
        @self.app.route('/api/move', methods=['POST'])
        def move():
            data = request.json
            joint = data.get('joint')
            position = data.get('position')
            
            if joint in self.limits:
                actual_pos = self.move_joint(joint, position)
                return jsonify({"success": True, "position": actual_pos})
            return jsonify({"success": False, "error": "Invalid joint"})
        
        @self.app.route('/api/status')
        def status():
            return jsonify({
                "positions": self.positions,
                "joint_states": self.joint_states,
                "limits": self.limits
            })
        
        @self.app.route('/api/preset/<preset>')
        def preset(preset):
            presets = {
                "home": {j: 0.0 for j in self.limits.keys()},
                "ready": {"Joint01": 0.0, "Joint02": -0.5, "Joint03": 1.0, "Joint04": 0.0, "Joint05": -0.5, "Joint06": 0.0, "Gripper": 0.0},
                "wave": {"Joint01": 0.5, "Joint02": -0.3, "Joint03": 1.2, "Joint04": 0.0, "Joint05": -0.9, "Joint06": 0.0, "Gripper": 0.3}
            }
            
            if preset in presets:
                for joint, pos in presets[preset].items():
                    self.move_joint(joint, pos)
                return jsonify({"success": True})
            return jsonify({"success": False, "error": "Unknown preset"})
    
    def run(self):
        """Start web server"""
        print("🌐 Z1 Web GUI starting...")
        print("📱 Open browser: http://localhost:8080")
        print("🛑 Press Ctrl+C to stop")
        
        # Start ROS spinner in background
        def ros_spin():
            rospy.spin()
        
        ros_thread = threading.Thread(target=ros_spin)
        ros_thread.daemon = True
        ros_thread.start()
        
        # Start Flask server
        self.app.run(host='0.0.0.0', port=8080, debug=False)

HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>Z1 Robot Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }
        .container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; }
        .joint-control { margin: 10px 0; padding: 15px; background: #f9f9f9; border-radius: 5px; }
        .slider { width: 100%; margin: 10px 0; }
        .preset-btn { margin: 5px; padding: 10px 20px; background: #007bff; color: white; border: none; border-radius: 5px; cursor: pointer; }
        .preset-btn:hover { background: #0056b3; }
        .status { margin: 10px 0; padding: 10px; background: #e9ecef; border-radius: 5px; font-family: monospace; }
        .title { text-align: center; color: #333; }
    </style>
</head>
<body>
    <div class="container">
        <h1 class="title">🤖 Unitree Z1 Web Control</h1>
        
        <div>
            <h3>Quick Presets</h3>
            <button class="preset-btn" onclick="setPreset('home')">🏠 Home</button>
            <button class="preset-btn" onclick="setPreset('ready')">⚡ Ready</button>
            <button class="preset-btn" onclick="setPreset('wave')">👋 Wave</button>
        </div>
        
        <div id="controls"></div>
        
        <div class="status">
            <h3>Robot Status</h3>
            <div id="status">Connecting...</div>
        </div>
    </div>

    <script>
        const joints = ['Joint01', 'Joint02', 'Joint03', 'Joint04', 'Joint05', 'Joint06', 'Gripper'];
        const jointNames = ['Base', 'Shoulder', 'Elbow', 'Forearm', 'Wrist Pitch', 'Wrist Roll', 'Gripper'];
        
        // Create controls
        function createControls() {
            const container = document.getElementById('controls');
            container.innerHTML = '<h3>Joint Controls</h3>';
            
            joints.forEach((joint, i) => {
                const div = document.createElement('div');
                div.className = 'joint-control';
                div.innerHTML = `
                    <label><strong>${jointNames[i]} (${joint})</strong></label><br>
                    <input type="range" class="slider" id="${joint}" 
                           min="-1.2" max="1.2" step="0.01" value="0"
                           oninput="moveJoint('${joint}', this.value)">
                    <span id="${joint}_value">0.00</span>
                `;
                container.appendChild(div);
            });
        }
        
        // Move joint
        function moveJoint(joint, position) {
            document.getElementById(joint + '_value').textContent = parseFloat(position).toFixed(2);
            
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
        
        // Update status
        function updateStatus() {
            fetch('/api/status')
                .then(response => response.json())
                .then(data => {
                    // Update sliders
                    Object.keys(data.positions).forEach(joint => {
                        const slider = document.getElementById(joint);
                        const value = document.getElementById(joint + '_value');
                        if (slider && value) {
                            slider.value = data.positions[joint];
                            value.textContent = data.positions[joint].toFixed(2);
                        }
                    });
                    
                    // Update status display
                    document.getElementById('status').innerHTML = 
                        'Current Positions:<br>' + 
                        Object.keys(data.joint_states).map(joint => 
                            `${joint}: ${data.joint_states[joint].toFixed(2)}`
                        ).join('<br>');
                });
        }
        
        // Initialize
        createControls();
        updateStatus();
        setInterval(updateStatus, 1000); // Update every second
    </script>
</body>
</html>
'''

if __name__ == "__main__":
    try:
        gui = Z1WebGUI()
        gui.run()
    except KeyboardInterrupt:
        print("\n🛑 Web GUI stopped")
    except Exception as e:
        print(f"❌ Error: {e}")