#!/usr/bin/env python3

import rospy
import json
from std_msgs.msg import String
from unitree_legged_msgs.msg import MotorCmd

class ROS2Bridge:
    def __init__(self):
        rospy.init_node('ros2_bridge')
        
        # ROS1 publishers
        self.motor_pub = rospy.Publisher('/z1_gazebo/joint1_controller/command', MotorCmd, queue_size=10)
        
        # Mock ROS2 interface (would connect to actual ROS2 in real implementation)
        self.ros2_sub = rospy.Subscriber('/ros2_commands', String, self.ros2_callback)
        
        rospy.loginfo("🔗 ROS2 Bridge initialized (experimental)")
    
    def ros2_callback(self, msg):
        """Handle ROS2 commands and translate to ROS1"""
        try:
            cmd_data = json.loads(msg.data)
            
            # Translate ROS2 command to ROS1 MotorCmd
            motor_cmd = MotorCmd()
            motor_cmd.mode = 10  # Position mode
            motor_cmd.q = cmd_data.get('position', 0.0)
            motor_cmd.dq = cmd_data.get('velocity', 0.0)
            motor_cmd.tau = cmd_data.get('torque', 0.0)
            motor_cmd.Kp = 20.0
            motor_cmd.Kd = 0.5
            
            self.motor_pub.publish(motor_cmd)
            rospy.loginfo(f"Bridged ROS2 command: {cmd_data}")
            
        except Exception as e:
            rospy.logerr(f"Bridge error: {e}")
    
    def run(self):
        rospy.loginfo("ROS2 Bridge running...")
        rospy.spin()

if __name__ == '__main__':
    bridge = ROS2Bridge()
    bridge.run()