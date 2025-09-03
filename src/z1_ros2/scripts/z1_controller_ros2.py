#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import numpy as np

class Z1ControllerROS2(Node):
    def __init__(self):
        super().__init__('z1_controller_ros2')
        
        # Publishers
        self.joint_pub = self.create_publisher(Float64MultiArray, '/z1/joint_commands', 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/z1/gripper_command', 10)
        
        # Subscribers
        self.joint_sub = self.create_subscription(JointState, '/z1/joint_states', self.joint_callback, 10)
        self.cmd_sub = self.create_subscription(Twist, '/z1/cmd_vel', self.cmd_callback, 10)
        
        # Joint positions
        self.joint_positions = [0.0] * 6
        self.gripper_position = 0.0
        
        # Timer for control loop
        self.timer = self.create_timer(0.01, self.control_loop)  # 100Hz
        
        self.get_logger().info('Z1 ROS2 Controller initialized')
    
    def joint_callback(self, msg):
        """Update current joint positions"""
        if len(msg.position) >= 6:
            self.joint_positions = list(msg.position[:6])
    
    def cmd_callback(self, msg):
        """Handle velocity commands"""
        # Convert Twist to joint velocities (simplified)
        joint_cmd = Float64MultiArray()
        joint_cmd.data = [
            msg.linear.x * 0.1,   # Joint 1
            msg.linear.y * 0.1,   # Joint 2
            msg.linear.z * 0.1,   # Joint 3
            msg.angular.x * 0.1,  # Joint 4
            msg.angular.y * 0.1,  # Joint 5
            msg.angular.z * 0.1   # Joint 6
        ]
        self.joint_pub.publish(joint_cmd)
    
    def control_loop(self):
        """Main control loop"""
        # Publish current joint commands
        joint_cmd = Float64MultiArray()
        joint_cmd.data = self.joint_positions
        self.joint_pub.publish(joint_cmd)
    
    def home_position(self):
        """Move to home position"""
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.get_logger().info('Moving to home position')

def main(args=None):
    rclpy.init(args=args)
    controller = Z1ControllerROS2()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()