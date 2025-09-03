#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from flask import Flask, render_template_string, request, jsonify
from flask_socketio import SocketIO, emit
from geometry_msgs.msg import Twist
import threading
import json

class Z1WebROS2(Node):
    def __init__(self):
        super().__init__('z1_web_ros2')
        
        # Publisher for robot commands
        self.cmd_pub = self.create_publisher(Twist, '/z1/cmd_vel', 10)
        
        self.get_logger().info('Z1 ROS2 Web Interface initialized')

# Flask app
app = Flask(__name__)
app.config['SECRET_KEY'] = 'z1_ros2_secret'
socketio = SocketIO(app, cors_allowed_origins="*")

# Global node reference
z1_node = None

# Modern React-style web interface
WEB_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Z1 ROS2 Control</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
    <script src="https://unpkg.com/react@18/umd/react.development.js"></script>
    <script src="https://unpkg.com/react-dom@18/umd/react-dom.development.js"></script>
    <script src="https://unpkg.com/@babel/standalone/babel.min.js"></script>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { 
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white; 
            min-height: 100vh;
        }
        .container { 
            max-width: 1400px; 
            margin: 0 auto; 
            padding: 20px; 
        }
        .header {
            text-align: center;
            margin-bottom: 30px;
            background: rgba(255,255,255,0.1);
            padding: 20px;
            border-radius: 15px;
            backdrop-filter: blur(10px);
        }
        .control-grid { 
            display: grid; 
            grid-template-columns: repeat(auto-fit, minmax(350px, 1fr)); 
            gap: 25px; 
        }
        .control-panel { 
            background: rgba(255,255,255,0.15); 
            border-radius: 15px; 
            padding: 25px;
            backdrop-filter: blur(10px);
            border: 1px solid rgba(255,255,255,0.2);
            transition: transform 0.3s ease;
        }
        .control-panel:hover {
            transform: translateY(-5px);
        }
        .btn { 
            background: linear-gradient(45deg, #ff6b6b, #ee5a24);
            color: white; 
            border: none; 
            padding: 12px 24px; 
            border-radius: 8px; 
            cursor: pointer;
            font-size: 14px;
            font-weight: 600;
            margin: 5px;
            transition: all 0.3s ease;
            box-shadow: 0 4px 15px rgba(0,0,0,0.2);
        }
        .btn:hover { 
            background: linear-gradient(45deg, #ee5a24, #ff6b6b);
            transform: translateY(-2px);
            box-shadow: 0 6px 20px rgba(0,0,0,0.3);
        }
        .btn:active {
            transform: translateY(0);
        }
        .status { 
            background: rgba(0,0,0,0.3); 
            padding: 15px; 
            border-radius: 8px; 
            margin: 15px 0;
            border-left: 4px solid #00d4aa;
            font-family: 'Courier New', monospace;
        }
        .joystick-container {
            display: flex;
            justify-content: center;
            align-items: center;
            height: 200px;
        }
        .joystick {
            width: 150px;
            height: 150px;
            border: 3px solid rgba(255,255,255,0.3);
            border-radius: 50%;
            position: relative;
            background: rgba(255,255,255,0.1);
        }
        .joystick-knob {
            width: 40px;
            height: 40px;
            background: linear-gradient(45deg, #00d4aa, #01a3a4);
            border-radius: 50%;
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            cursor: pointer;
            box-shadow: 0 4px 15px rgba(0,0,0,0.3);
        }
        .joint-slider {
            width: 100%;
            margin: 10px 0;
            -webkit-appearance: none;
            height: 8px;
            border-radius: 5px;
            background: rgba(255,255,255,0.2);
            outline: none;
        }
        .joint-slider::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 20px;
            height: 20px;
            border-radius: 50%;
            background: linear-gradient(45deg, #00d4aa, #01a3a4);
            cursor: pointer;
            box-shadow: 0 2px 10px rgba(0,0,0,0.3);
        }
        h1 { font-size: 2.5em; margin-bottom: 10px; }
        h3 { color: #00d4aa; margin-bottom: 15px; }
        .emoji { font-size: 1.5em; margin-right: 10px; }
    </style>
</head>
<body>
    <div id="root"></div>
    
    <script type="text/babel">
        const { useState, useEffect } = React;
        
        function Z1Controller() {
            const [status, setStatus] = useState('Connecting...');
            const [jointStatus, setJointStatus] = useState('Ready');
            const [socket, setSocket] = useState(null);
            
            useEffect(() => {
                const newSocket = io();
                setSocket(newSocket);
                
                newSocket.on('connect', () => {
                    setStatus('Connected to Z1 ROS2');
                });
                
                newSocket.on('status', (data) => {
                    setJointStatus(data.message);
                });
                
                return () => newSocket.close();
            }, []);
            
            const sendCommand = (action, params = {}) => {
                if (socket) {
                    socket.emit('command', { action, ...params });
                    setJointStatus(`Executing: ${action}`);
                }
            };
            
            const sendTwist = (linear, angular) => {
                if (socket) {
                    socket.emit('twist', { linear, angular });
                }
            };
            
            return (\n                <div className=\"container\">\n                    <div className=\"header\">\n                        <h1><span className=\"emoji\">🤖</span>Z1 ROS2 Control Interface</h1>\n                        <p>Next-generation robotic arm control with ROS2</p>\n                    </div>\n                    \n                    <div className=\"control-grid\">\n                        <div className=\"control-panel\">\n                            <h3><span className=\"emoji\">🎮</span>Quick Actions</h3>\n                            <button className=\"btn\" onClick={() => sendCommand('home')}>🏠 Home Position</button>\n                            <button className=\"btn\" onClick={() => sendCommand('wave')}>👋 Wave Gesture</button>\n                            <button className=\"btn\" onClick={() => sendCommand('stop')}>⛔ Emergency Stop</button>\n                            <div className=\"status\">{jointStatus}</div>\n                        </div>\n                        \n                        <div className=\"control-panel\">\n                            <h3><span className=\"emoji\">🕹️</span>Virtual Joystick</h3>\n                            <div className=\"joystick-container\">\n                                <div className=\"joystick\">\n                                    <div className=\"joystick-knob\"></div>\n                                </div>\n                            </div>\n                            <p>Drag to control base and shoulder joints</p>\n                        </div>\n                        \n                        <div className=\"control-panel\">\n                            <h3><span className=\"emoji\">⚙️</span>Joint Control</h3>\n                            {['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'].map((joint, i) => (\n                                <div key={i}>\n                                    <label>{joint}</label>\n                                    <input \n                                        type=\"range\" \n                                        className=\"joint-slider\"\n                                        min=\"-180\" \n                                        max=\"180\" \n                                        defaultValue=\"0\"\n                                        onChange={(e) => sendCommand('joint', {index: i, value: e.target.value})}\n                                    />\n                                </div>\n                            ))}\n                        </div>\n                        \n                        <div className=\"control-panel\">\n                            <h3><span className=\"emoji\">📊</span>System Status</h3>\n                            <div className=\"status\">{status}</div>\n                            <div className=\"status\">ROS2 Humble Active</div>\n                            <div className=\"status\">Gazebo Simulation Ready</div>\n                            <div className=\"status\">Controllers: Online</div>\n                        </div>\n                    </div>\n                </div>\n            );\n        }\n        \n        ReactDOM.render(<Z1Controller />, document.getElementById('root'));\n    </script>\n</body>\n</html>\n"""\n\n@app.route('/')\ndef index():\n    return render_template_string(WEB_TEMPLATE)\n\n@socketio.on('command')\ndef handle_command(data):\n    global z1_node\n    if z1_node:\n        action = data.get('action')\n        z1_node.get_logger().info(f'Web command: {action}')\n        \n        if action == 'home':\n            z1_node.home_position()\n        elif action == 'wave':\n            z1_node.wave_gesture()\n        elif action == 'stop':\n            # Send zero velocity\n            twist = Twist()\n            z1_node.cmd_pub.publish(twist)\n        \n        emit('status', {'message': f'Executed: {action}'})\n\n@socketio.on('twist')\ndef handle_twist(data):\n    global z1_node\n    if z1_node:\n        twist = Twist()\n        linear = data.get('linear', {})\n        angular = data.get('angular', {})\n        \n        twist.linear.x = linear.get('x', 0.0)\n        twist.linear.y = linear.get('y', 0.0)\n        twist.linear.z = linear.get('z', 0.0)\n        twist.angular.x = angular.get('x', 0.0)\n        twist.angular.y = angular.get('y', 0.0)\n        twist.angular.z = angular.get('z', 0.0)\n        \n        z1_node.cmd_pub.publish(twist)\n\ndef run_flask():\n    socketio.run(app, host='0.0.0.0', port=8080, debug=False)\n\ndef main(args=None):\n    global z1_node\n    \n    rclpy.init(args=args)\n    z1_node = Z1WebROS2()\n    \n    # Start Flask in separate thread\n    flask_thread = threading.Thread(target=run_flask)\n    flask_thread.daemon = True\n    flask_thread.start()\n    \n    z1_node.get_logger().info('Web interface available at http://localhost:8080')\n    \n    try:\n        rclpy.spin(z1_node)\n    except KeyboardInterrupt:\n        pass\n    finally:\n        z1_node.destroy_node()\n        rclpy.shutdown()\n\nif __name__ == '__main__':\n    main()