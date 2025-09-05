#!/usr/bin/env python3

"""
Z1 Drummer - Play drum beats and rhythms
"""

import rospy
import math
import time
from std_msgs.msg import Float64

class Z1Drummer:
    def __init__(self):
        rospy.init_node("z1_drummer")
        
        self.pubs = {}
        for i in range(1, 7):
            topic = f"/z1_gazebo/Joint0{i}_controller/command"
            self.pubs[f"Joint0{i}"] = rospy.Publisher(topic, Float64, queue_size=1)
        
        self.gripper_pub = rospy.Publisher("/z1_gazebo/Gripper_controller/command", Float64, queue_size=1)
        
    def move_to(self, positions, duration=0.2):
        """Quick drumming movements"""
        steps = int(duration * 50)
        for step in range(steps):
            for joint, pos in positions.items():
                if joint in self.pubs:
                    self.pubs[joint].publish(Float64(pos))
                elif joint == "Gripper":
                    self.gripper_pub.publish(Float64(pos))
            time.sleep(0.02)
    
    def hit_snare(self):
        """Hit snare drum (center)"""
        # Raise stick
        up_pos = {"Joint01": 0.0, "Joint02": -0.2, "Joint03": 0.8, "Joint04": 0.0, "Joint05": -0.6, "Joint06": 0.0, "Gripper": 0.6}
        self.move_to(up_pos, 0.1)
        
        # Strike down
        down_pos = {"Joint01": 0.0, "Joint02": -0.5, "Joint03": 1.2, "Joint04": 0.0, "Joint05": -0.7, "Joint06": 0.0, "Gripper": 0.6}
        self.move_to(down_pos, 0.05)
        
        # Quick return
        self.move_to(up_pos, 0.05)
    
    def hit_hihat(self):
        """Hit hi-hat (left side)"""
        up_pos = {"Joint01": -0.4, "Joint02": -0.2, "Joint03": 0.8, "Joint04": 0.0, "Joint05": -0.6, "Joint06": 0.0, "Gripper": 0.6}
        self.move_to(up_pos, 0.1)
        
        down_pos = {"Joint01": -0.4, "Joint02": -0.5, "Joint03": 1.2, "Joint04": 0.0, "Joint05": -0.7, "Joint06": 0.0, "Gripper": 0.6}
        self.move_to(down_pos, 0.05)
        
        self.move_to(up_pos, 0.05)
    
    def hit_crash(self):
        """Hit crash cymbal (right side)"""
        up_pos = {"Joint01": 0.4, "Joint02": -0.1, "Joint03": 0.6, "Joint04": 0.0, "Joint05": -0.5, "Joint06": 0.0, "Gripper": 0.6}
        self.move_to(up_pos, 0.1)
        
        down_pos = {"Joint01": 0.4, "Joint02": -0.4, "Joint03": 1.0, "Joint04": 0.0, "Joint05": -0.6, "Joint06": 0.0, "Gripper": 0.6}
        self.move_to(down_pos, 0.05)
        
        self.move_to(up_pos, 0.1)
    
    def basic_beat(self):
        """Play basic rock beat"""
        rospy.loginfo("🥁 Playing basic rock beat...")
        
        for measure in range(4):
            # Beat 1: Kick + Hi-hat
            self.hit_hihat()
            time.sleep(0.2)
            
            # Beat 2: Hi-hat
            self.hit_hihat()
            time.sleep(0.2)
            
            # Beat 3: Snare + Hi-hat
            self.hit_snare()
            time.sleep(0.1)
            self.hit_hihat()
            time.sleep(0.1)
            
            # Beat 4: Hi-hat
            self.hit_hihat()
            time.sleep(0.2)
    
    def drum_solo(self):
        """Energetic drum solo"""
        rospy.loginfo("🔥 Drum solo time!")
        
        # Fast alternating hits
        for i in range(16):
            if i % 4 == 0:
                self.hit_crash()
            elif i % 2 == 0:
                self.hit_snare()
            else:
                self.hit_hihat()
            time.sleep(0.1)
        
        # Build up
        for i in range(8):
            self.hit_snare()
            time.sleep(0.05)
        
        # Finale crash
        self.hit_crash()
        time.sleep(1.0)
    
    def jazz_shuffle(self):
        """Play jazz shuffle rhythm"""
        rospy.loginfo("🎷 Jazz shuffle rhythm...")
        
        for measure in range(3):
            # Swing rhythm
            self.hit_hihat()
            time.sleep(0.15)
            self.hit_hihat()
            time.sleep(0.1)
            
            self.hit_snare()
            time.sleep(0.15)
            self.hit_hihat()
            time.sleep(0.1)
            
            self.hit_hihat()
            time.sleep(0.15)
            self.hit_hihat()
            time.sleep(0.1)
            
            self.hit_snare()
            time.sleep(0.25)
    
    def paradiddle_exercise(self):
        """Practice paradiddles (RLRR LRLL)"""
        rospy.loginfo("🎯 Paradiddle exercise...")
        
        # RLRR pattern
        for rep in range(4):
            self.hit_snare()  # R
            time.sleep(0.1)
            self.hit_hihat()  # L
            time.sleep(0.1)
            self.hit_snare()  # R
            time.sleep(0.1)
            self.hit_snare()  # R
            time.sleep(0.1)
            
            self.hit_hihat()  # L
            time.sleep(0.1)
            self.hit_snare()  # R
            time.sleep(0.1)
            self.hit_hihat()  # L
            time.sleep(0.1)
            self.hit_hihat()  # L
            time.sleep(0.1)
    
    def run(self):
        rospy.loginfo("🥁 Z1 Drummer ready to rock!")
        
        # Drumming session
        rospy.loginfo("🎵 Starting drum session...")
        
        # Warm up
        self.paradiddle_exercise()
        time.sleep(1)
        
        # Main performance
        self.basic_beat()
        time.sleep(1)
        
        self.jazz_shuffle()
        time.sleep(1)
        
        self.drum_solo()
        
        # Final bow with sticks up
        bow = {"Joint01": 0.0, "Joint02": 0.2, "Joint03": 0.3, "Joint04": 0.0, "Joint05": -0.3, "Joint06": 0.0, "Gripper": 0.6}
        self.move_to(bow, 2.0)
        time.sleep(2)
        
        rospy.loginfo("🎤 *drops mic* Drumming complete!")

if __name__ == "__main__":
    try:
        drummer = Z1Drummer()
        drummer.run()
    except rospy.ROSInterruptException:
        pass