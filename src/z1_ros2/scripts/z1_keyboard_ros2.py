#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

class Z1KeyboardROS2(Node):
    def __init__(self):
        super().__init__('z1_keyboard_ros2')
        self.cmd_pub = self.create_publisher(Twist, '/z1/cmd_vel', 10)
        self.get_logger().info('Z1 ROS2 Keyboard Control Ready')
        self.print_controls()
    
    def print_controls(self):
        print("""
🎮 Z1 ROS2 Keyboard Controls:
---------------------------
WASD: Base/Shoulder joints
ZE:   Elbow joint
RF:   Forearm joint  
TG:   Wrist pitch
YH:   Wrist roll
Space/X: Gripper open/close
ESC:  Exit
        """)
    
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def run(self):
        self.settings = termios.tcgetattr(sys.stdin)
        
        try:
            while rclpy.ok():
                key = self.get_key()
                
                twist = Twist()
                
                # Joint control mapping
                if key == 'w':
                    twist.linear.x = 1.0
                elif key == 's':
                    twist.linear.x = -1.0
                elif key == 'a':
                    twist.linear.y = 1.0
                elif key == 'd':
                    twist.linear.y = -1.0
                elif key == 'z':
                    twist.linear.z = 1.0
                elif key == 'e':
                    twist.linear.z = -1.0
                elif key == 'r':
                    twist.angular.x = 1.0
                elif key == 'f':
                    twist.angular.x = -1.0
                elif key == 't':
                    twist.angular.y = 1.0
                elif key == 'g':
                    twist.angular.y = -1.0
                elif key == 'y':
                    twist.angular.z = 1.0
                elif key == 'h':
                    twist.angular.z = -1.0
                elif key == '\x1b':  # ESC
                    break
                
                self.cmd_pub.publish(twist)
                
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    keyboard_node = Z1KeyboardROS2()
    keyboard_node.run()
    keyboard_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()