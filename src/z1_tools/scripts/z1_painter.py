#!/usr/bin/env python3

"""
Z1 Painter - Create artistic drawings and paintings
"""

import rospy
import math
import time
from std_msgs.msg import Float64

class Z1Painter:
    def __init__(self):
        rospy.init_node("z1_painter")
        
        self.pubs = {}
        for i in range(1, 7):
            topic = f"/z1_gazebo/Joint0{i}_controller/command"
            self.pubs[f"Joint0{i}"] = rospy.Publisher(topic, Float64, queue_size=1)
        
        self.gripper_pub = rospy.Publisher("/z1_gazebo/Gripper_controller/command", Float64, queue_size=1)
        
    def move_to(self, positions, duration=2.0):
        """Move to joint positions"""
        steps = int(duration * 50)
        for step in range(steps):
            for joint, pos in positions.items():
                if joint in self.pubs:
                    self.pubs[joint].publish(Float64(pos))
                elif joint == "Gripper":
                    self.gripper_pub.publish(Float64(pos))
            time.sleep(0.02)
    
    def paint_heart(self):
        """Paint a heart shape"""
        rospy.loginfo("🎨 Painting a heart...")
        
        # Heart equation: x = 16sin³(t), y = 13cos(t) - 5cos(2t) - 2cos(3t) - cos(4t)
        for t in range(0, 628, 10):  # 0 to 2π in steps
            angle = t / 100.0
            
            # Scale down for robot workspace
            x_offset = 0.02 * math.sin(angle)**3
            y_offset = 0.01 * (13*math.cos(angle) - 5*math.cos(2*angle) - 2*math.cos(3*angle) - math.cos(4*angle))
            
            positions = {
                "Joint01": x_offset,
                "Joint02": -0.3 + y_offset * 0.1,
                "Joint03": 1.0,
                "Joint04": 0.0,
                "Joint05": -0.7,
                "Joint06": 0.0,
                "Gripper": 0.3
            }
            
            self.move_to(positions, 0.1)
    
    def paint_spiral(self):
        """Paint a spiral pattern"""
        rospy.loginfo("🌀 Painting a spiral...")
        
        for i in range(100):
            angle = i * 0.2
            radius = i * 0.002
            
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            
            positions = {
                "Joint01": x,
                "Joint02": -0.3 + y,
                "Joint03": 1.0,
                "Joint04": 0.0,
                "Joint05": -0.7,
                "Joint06": angle * 0.1,
                "Gripper": 0.3
            }
            
            self.move_to(positions, 0.05)
    
    def run(self):
        rospy.loginfo("🎨 Z1 Painter ready!")
        
        # Paint sequence
        self.paint_heart()
        time.sleep(2)
        self.paint_spiral()
        
        rospy.loginfo("🖼️ Masterpiece complete!")

if __name__ == "__main__":
    try:
        painter = Z1Painter()
        painter.run()
    except rospy.ROSInterruptException:
        pass