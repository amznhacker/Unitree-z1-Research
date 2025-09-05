#!/usr/bin/env python3

"""
Z1 Yoga Instructor - Demonstrate yoga poses and flows
"""

import rospy
import math
import time
from std_msgs.msg import Float64

class Z1YogaInstructor:
    def __init__(self):
        rospy.init_node("z1_yoga_instructor")
        
        self.pubs = {}
        for i in range(1, 7):
            topic = f"/z1_gazebo/Joint0{i}_controller/command"
            self.pubs[f"Joint0{i}"] = rospy.Publisher(topic, Float64, queue_size=1)
        
        self.gripper_pub = rospy.Publisher("/z1_gazebo/Gripper_controller/command", Float64, queue_size=1)
        
    def move_to(self, positions, duration=3.0):
        """Slow, mindful movements"""
        steps = int(duration * 50)
        for step in range(steps):
            for joint, pos in positions.items():
                if joint in self.pubs:
                    self.pubs[joint].publish(Float64(pos))
                elif joint == "Gripper":
                    self.gripper_pub.publish(Float64(pos))
            time.sleep(0.02)
    
    def mountain_pose(self):
        """Tadasana - Mountain Pose"""
        rospy.loginfo("🏔️ Mountain Pose - Find your center...")
        
        pose = {
            "Joint01": 0.0,
            "Joint02": 0.0,
            "Joint03": 0.0,
            "Joint04": 0.0,
            "Joint05": 0.0,
            "Joint06": 0.0,
            "Gripper": 0.0
        }
        
        self.move_to(pose, 4.0)
        time.sleep(3.0)  # Hold the pose
    
    def tree_pose(self):
        """Vrksasana - Tree Pose"""
        rospy.loginfo("🌳 Tree Pose - Balance and stability...")
        
        pose = {
            "Joint01": 0.0,
            "Joint02": -0.3,
            "Joint03": 0.8,
            "Joint04": 0.5,
            "Joint05": -0.5,
            "Joint06": 0.0,
            "Gripper": 0.3  # Hands in prayer position
        }
        
        self.move_to(pose, 4.0)
        time.sleep(4.0)
    
    def warrior_pose(self):
        """Virabhadrasana - Warrior Pose"""
        rospy.loginfo("⚔️ Warrior Pose - Strength and focus...")
        
        pose = {
            "Joint01": 0.6,
            "Joint02": -0.2,
            "Joint03": 0.5,
            "Joint04": 0.0,
            "Joint05": -0.3,
            "Joint06": 0.0,
            "Gripper": 0.6  # Arms extended
        }
        
        self.move_to(pose, 4.0)
        time.sleep(4.0)
        
        # Switch sides
        pose["Joint01"] = -0.6
        self.move_to(pose, 4.0)
        time.sleep(4.0)
    
    def sun_salutation(self):
        """Surya Namaskara - Sun Salutation Flow"""
        rospy.loginfo("☀️ Sun Salutation - Flowing sequence...")
        
        # Mountain Pose
        mountain = {"Joint01": 0.0, "Joint02": 0.0, "Joint03": 0.0, "Joint04": 0.0, "Joint05": 0.0, "Joint06": 0.0, "Gripper": 0.0}
        self.move_to(mountain, 3.0)
        time.sleep(2.0)
        
        # Upward Salute
        upward = {"Joint01": 0.0, "Joint02": 0.3, "Joint03": 0.0, "Joint04": 0.0, "Joint05": -0.3, "Joint06": 0.0, "Gripper": 0.6}
        self.move_to(upward, 3.0)
        time.sleep(2.0)
        
        # Forward Fold
        fold = {"Joint01": 0.0, "Joint02": -0.8, "Joint03": 1.5, "Joint04": 0.0, "Joint05": -0.7, "Joint06": 0.0, "Gripper": 0.0}
        self.move_to(fold, 4.0)
        time.sleep(3.0)
        
        # Half Lift
        half_lift = {"Joint01": 0.0, "Joint02": -0.4, "Joint03": 1.0, "Joint04": 0.0, "Joint05": -0.6, "Joint06": 0.0, "Gripper": 0.3}
        self.move_to(half_lift, 3.0)
        time.sleep(2.0)
        
        # Return to Mountain
        self.move_to(mountain, 4.0)
        time.sleep(2.0)
    
    def eagle_pose(self):
        """Garudasana - Eagle Pose"""
        rospy.loginfo("🦅 Eagle Pose - Concentration and balance...")
        
        pose = {
            "Joint01": 0.2,
            "Joint02": -0.4,
            "Joint03": 1.2,
            "Joint04": -0.3,
            "Joint05": -0.8,
            "Joint06": 0.2,
            "Gripper": 0.4
        }
        
        self.move_to(pose, 5.0)
        time.sleep(4.0)
    
    def child_pose(self):
        """Balasana - Child's Pose"""
        rospy.loginfo("🧘 Child's Pose - Rest and restoration...")
        
        pose = {
            "Joint01": 0.0,
            "Joint02": -0.9,
            "Joint03": 1.8,
            "Joint04": 0.0,
            "Joint05": -0.9,
            "Joint06": 0.0,
            "Gripper": 0.0
        }
        
        self.move_to(pose, 5.0)
        time.sleep(5.0)  # Longer rest
    
    def breathing_exercise(self):
        """Pranayama - Breathing exercise with movement"""
        rospy.loginfo("🌬️ Breathing exercise - Inhale... Exhale...")
        
        for breath in range(5):
            # Inhale - expand
            inhale_pose = {
                "Joint01": 0.0,
                "Joint02": 0.2,
                "Joint03": 0.3,
                "Joint04": 0.0,
                "Joint05": -0.5,
                "Joint06": 0.0,
                "Gripper": 0.6
            }
            self.move_to(inhale_pose, 4.0)  # 4 seconds inhale
            
            # Exhale - contract
            exhale_pose = {
                "Joint01": 0.0,
                "Joint02": -0.2,
                "Joint03": 0.8,
                "Joint04": 0.0,
                "Joint05": -0.6,
                "Joint06": 0.0,
                "Gripper": 0.0
            }
            self.move_to(exhale_pose, 4.0)  # 4 seconds exhale
    
    def run(self):
        rospy.loginfo("🧘‍♀️ Z1 Yoga Instructor ready for practice!")
        rospy.loginfo("🕯️ Welcome to your yoga session. Let's begin...")
        
        # Yoga session sequence
        self.mountain_pose()
        
        rospy.loginfo("💨 Let's start with some breathing...")
        self.breathing_exercise()
        
        rospy.loginfo("🌅 Moving into Sun Salutation...")
        self.sun_salutation()
        
        rospy.loginfo("🌳 Standing poses for strength...")
        self.tree_pose()
        
        self.warrior_pose()
        
        rospy.loginfo("🦅 Balance challenge...")
        self.eagle_pose()
        
        rospy.loginfo("😌 Time to rest...")
        self.child_pose()
        
        # Final relaxation
        rospy.loginfo("🙏 Namaste - Thank you for practicing with me")
        self.mountain_pose()
        
        rospy.loginfo("✨ Yoga session complete. Peace and light to you!")

if __name__ == "__main__":
    try:
        instructor = Z1YogaInstructor()
        instructor.run()
    except rospy.ROSInterruptException:
        pass