#!/usr/bin/env python3

"""
Z1 Chef - Cooking and food preparation demonstrations
"""

import rospy
import math
import time
from std_msgs.msg import Float64

class Z1Chef:
    def __init__(self):
        rospy.init_node("z1_chef")
        
        self.pubs = {}
        for i in range(1, 7):
            topic = f"/z1_gazebo/Joint0{i}_controller/command"
            self.pubs[f"Joint0{i}"] = rospy.Publisher(topic, Float64, queue_size=1)
        
        self.gripper_pub = rospy.Publisher("/z1_gazebo/Gripper_controller/command", Float64, query_size=1)
        
    def move_to(self, positions, duration=2.0):
        """Smooth cooking movements"""
        steps = int(duration * 50)
        for step in range(steps):
            for joint, pos in positions.items():
                if joint in self.pubs:
                    self.pubs[joint].publish(Float64(pos))
                elif joint == "Gripper":
                    self.gripper_pub.publish(Float64(pos))
            time.sleep(0.02)
    
    def chop_vegetables(self):
        """Simulate chopping motion"""
        rospy.loginfo("🔪 Chopping vegetables...")
        
        # Chopping motion - up and down
        for i in range(15):
            # Knife up
            up_pos = {
                "Joint01": 0.0,
                "Joint02": -0.3,
                "Joint03": 0.8,
                "Joint04": 0.0,
                "Joint05": -0.5,
                "Joint06": 0.0,
                "Gripper": 0.6
            }
            self.move_to(up_pos, 0.2)
            
            # Knife down (chop)
            down_pos = {
                "Joint01": 0.0,
                "Joint02": -0.6,
                "Joint03": 1.2,
                "Joint04": 0.0,
                "Joint05": -0.6,
                "Joint06": 0.0,
                "Gripper": 0.6
            }
            self.move_to(down_pos, 0.1)
    
    def stir_pot(self):
        """Simulate stirring motion"""
        rospy.loginfo("🥄 Stirring the pot...")
        
        # Circular stirring motion
        for i in range(20):
            angle = i * 0.314  # π/10
            
            stir_pos = {
                "Joint01": 0.1 * math.cos(angle),
                "Joint02": -0.5 + 0.05 * math.sin(angle),
                "Joint03": 1.0,
                "Joint04": 0.0,
                "Joint05": -0.5,
                "Joint06": angle * 0.2,
                "Gripper": 0.6
            }
            self.move_to(stir_pos, 0.2)
    
    def flip_pancake(self):
        """Simulate pancake flipping"""
        rospy.loginfo("🥞 Flipping pancakes...")
        
        for flip in range(3):
            # Slide under pancake
            slide_pos = {
                "Joint01": 0.0,
                "Joint02": -0.7,
                "Joint03": 1.3,
                "Joint04": 0.0,
                "Joint05": -0.6,
                "Joint06": 0.0,
                "Gripper": 0.0
            }
            self.move_to(slide_pos, 1.0)
            
            # Quick flip motion
            flip_pos = {
                "Joint01": 0.0,
                "Joint02": -0.2,
                "Joint03": 0.5,
                "Joint04": 0.0,
                "Joint05": -0.3,
                "Joint06": 1.0,
                "Gripper": 0.0
            }
            self.move_to(flip_pos, 0.5)
            
            # Return to pan
            self.move_to(slide_pos, 0.8)
            time.sleep(0.5)
    
    def season_dish(self):
        """Simulate seasoning/sprinkling"""
        rospy.loginfo("🧂 Adding seasoning...")
        
        # Sprinkling motion
        for i in range(10):
            # Shake motion
            shake_pos = {
                "Joint01": 0.2 * math.sin(i * 0.5),
                "Joint02": -0.3,
                "Joint03": 0.8,
                "Joint04": 0.0,
                "Joint05": -0.5,
                "Joint06": 0.3 * math.sin(i * 0.8),
                "Gripper": 0.3 + 0.2 * math.sin(i * 2)
            }
            self.move_to(shake_pos, 0.2)
    
    def plate_presentation(self):
        """Elegant plating movements"""
        rospy.loginfo("🍽️ Plating the dish...")
        
        # Graceful plating motions
        motions = [
            {"Joint01": -0.3, "Joint02": -0.4, "Joint03": 1.0, "Joint04": 0.2, "Joint05": -0.6, "Joint06": 0.0, "Gripper": 0.4},
            {"Joint01": 0.0, "Joint02": -0.3, "Joint03": 0.8, "Joint04": 0.0, "Joint05": -0.5, "Joint06": 0.0, "Gripper": 0.2},
            {"Joint01": 0.3, "Joint02": -0.4, "Joint03": 1.0, "Joint04": -0.2, "Joint05": -0.6, "Joint06": 0.0, "Gripper": 0.4},
        ]
        
        for motion in motions:
            self.move_to(motion, 1.5)
            time.sleep(0.5)
    
    def run(self):
        rospy.loginfo("👨‍🍳 Z1 Chef ready to cook!")
        
        # Cooking sequence
        rospy.loginfo("🍳 Starting cooking demonstration...")
        
        # Prep work
        self.chop_vegetables()
        time.sleep(1)
        
        # Cooking
        self.stir_pot()
        time.sleep(1)
        
        self.flip_pancake()
        time.sleep(1)
        
        # Finishing touches
        self.season_dish()
        time.sleep(1)
        
        self.plate_presentation()
        
        # Chef's bow
        bow = {"Joint01": 0.0, "Joint02": -0.8, "Joint03": 1.5, "Joint04": 0.0, "Joint05": -0.7, "Joint06": 0.0, "Gripper": 0.0}
        self.move_to(bow, 2.0)
        time.sleep(2)
        
        rospy.loginfo("🎉 Bon appétit! Cooking complete!")

if __name__ == "__main__":
    try:
        chef = Z1Chef()
        chef.run()
    except rospy.ROSInterruptException:
        pass