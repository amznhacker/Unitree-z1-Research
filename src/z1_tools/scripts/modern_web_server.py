#!/usr/bin/env python3

import rospy
from flask import Flask, render_template_string
from flask_socketio import SocketIO
import json

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# Modern React-style web interface
MODERN_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>Z1 Modern Control</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
    <style>
        body { font-family: 'Segoe UI', sans-serif; background: #1a1a1a; color: white; margin: 0; }
        .container { max-width: 1200px; margin: 0 auto; padding: 20px; }
        .control-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; }
        .control-panel { background: #2d2d2d; border-radius: 10px; padding: 20px; }
        .btn { background: #007acc; color: white; border: none; padding: 10px 20px; border-radius: 5px; cursor: pointer; }
        .btn:hover { background: #005a9e; }
        .status { background: #333; padding: 10px; border-radius: 5px; margin: 10px 0; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 Z1 Modern Control Interface</h1>
        <div class="control-grid">
            <div class="control-panel">
                <h3>Joint Control</h3>
                <button class="btn" onclick="sendCommand('home')">Home Position</button>
                <button class="btn" onclick="sendCommand('wave')">Wave</button>
                <div class="status" id="joint-status">Ready</div>
            </div>
            <div class="control-panel">
                <h3>AI Vision</h3>
                <button class="btn" onclick="sendCommand('detect')">Detect Objects</button>
                <button class="btn" onclick="sendCommand('track')">Track Target</button>
                <div class="status" id="vision-status">Standby</div>
            </div>
        </div>
    </div>
    <script>
        const socket = io();
        function sendCommand(cmd) {
            socket.emit('command', {action: cmd});
            document.getElementById('joint-status').innerText = 'Executing: ' + cmd;
        }
        socket.on('status', (data) => {
            document.getElementById('vision-status').innerText = data.message;
        });
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(MODERN_TEMPLATE)

@socketio.on('command')
def handle_command(data):
    rospy.loginfo(f"Modern command: {data['action']}")
    socketio.emit('status', {'message': f"Executed: {data['action']}"})

if __name__ == '__main__':
    rospy.init_node('modern_web_server')
    socketio.run(app, host='0.0.0.0', port=8080, debug=True)