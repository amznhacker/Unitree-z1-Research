#!/usr/bin/env python3

"""
Z1 Dancer - Choreographed dance routines
"""

import rospy
import math
import time
from std_msgs.msg import Float64

class Z1Dancer:
    def __init__(self):
        rospy.init_node("z1_dancer")
        
        self.pubs = {}
        for i in range(1, 7):
            topic = f"/z1_gazebo/Joint0{i}_controller/command"
            self.pubs[f"Joint0{i}"] = rospy.Publisher(topic, Float64, queue_size=1)
        
        self.gripper_pub = rospy.Publisher("/z1_gazebo/Gripper_controller/command", Float64, queue_size=1)
        
    def move_to(self, positions, duration=1.0):
        """Smooth movement to positions"""
        steps = int(duration * 50)
        for step in range(steps):
            for joint, pos in positions.items():
                if joint in self.pubs:
                    self.pubs[joint].publish(Float64(pos))
                elif joint == "Gripper":
                    self.gripper_pub.publish(Float64(pos))
            time.sleep(0.02)
    
    def robot_dance(self):
        """Classic robot dance moves"""
        rospy.loginfo("🤖 Performing robot dance...")
        
        moves = [
            {"Joint01": 0.5, "Joint02": -0.3, "Joint03": 1.2, "Joint04": 0.0, "Joint05": -0.9, "Joint06": 0.0, "Gripper": 0.0},
            {"Joint01": -0.5, "Joint02": -0.3, "Joint03": 1.2, "Joint04": 0.0, "Joint05": -0.9, "Joint06": 0.0, "Gripper": 0.3},
            {"Joint01": 0.0, "Joint02": -0.8, "Joint03": 1.5, "Joint04": 0.5, "Joint05": -0.7, "Joint06": 0.5, "Gripper": 0.0},
            {"Joint01": 0.0, "Joint02": -0.8, "Joint03": 1.5, "Joint04": -0.5, "Joint05": -0.7, "Joint06": -0.5, "Gripper": 0.3},
        ]
        
        for move in moves * 3:  # Repeat 3 times
            self.move_to(move, 0.8)
    
    def wave_dance(self):
        """Flowing wave-like movements"""
        rospy.loginfo("🌊 Performing wave dance...")
        
        for i in range(50):
            t = i * 0.2
            
            positions = {
                "Joint01": 0.3 * math.sin(t),
                "Joint02": -0.4 + 0.2 * math.sin(t + 1),
                "Joint03": 1.0 + 0.3 * math.sin(t + 2),
                "Joint04": 0.2 * math.sin(t + 3),
                "Joint05": -0.6 + 0.2 * math.sin(t + 4),
                "Joint06": 0.3 * math.sin(t + 5),
                "Gripper": 0.3 * (1 + math.sin(t * 2))
            }
            
            self.move_to(positions, 0.1)
    
    def breakdance_freeze(self):
        """Dramatic freeze poses"""
        rospy.loginfo("🕺 Breakdance freeze poses...")
        
        freezes = [
            {"Joint01": 0.8, "Joint02": -0.9, "Joint03": 1.8, "Joint04": 0.3, "Joint05": -0.9, "Joint06": 0.5, "Gripper": 0.6},
            {"Joint01": -0.8, "Joint02": -0.2, "Joint03": 0.5, "Joint04": -0.3, "Joint05": -0.3, "Joint06": -0.5, "Gripper": 0.0},
            {"Joint01": 0.0, "Joint02": -1.0, "Joint03": 2.0, "Joint04": 0.0, "Joint05": -1.0, "Joint06": 0.0, "Gripper": 0.3},
        ]
        
        for freeze in freezes:
            self.move_to(freeze, 1.5)
            time.sleep(2.0)  # Hold the pose
    
    def run(self):
        rospy.loginfo("💃 Z1 Dancer ready to perform!")
        
        # Dance sequence
        self.robot_dance()
        time.sleep(1)
        self.wave_dance()
        time.sleep(1)
        self.breakdance_freeze()
        
        # Return to home
        home = {f"Joint0{i}": 0.0 for i in range(1, 7)}
        home["Gripper"] = 0.0
        self.move_to(home, 3.0)
        
        rospy.loginfo("🎭 Dance performance complete!")

if __name__ == "__main__":
    try:
        dancer = Z1Dancer()
        dancer.run()
    except rospy.ROSInterruptException:
        pass