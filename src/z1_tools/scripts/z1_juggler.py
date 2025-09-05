#!/usr/bin/env python3

"""
Z1 Juggler - Simulate juggling motions
"""

import rospy
import math
import time
from std_msgs.msg import Float64

class Z1Juggler:
    def __init__(self):
        rospy.init_node("z1_juggler")
        
        self.pubs = {}
        for i in range(1, 7):
            topic = f"/z1_gazebo/Joint0{i}_controller/command"
            self.pubs[f"Joint0{i}"] = rospy.Publisher(topic, Float64, queue_size=1)
        
        self.gripper_pub = rospy.Publisher("/z1_gazebo/Gripper_controller/command", Float64, queue_size=1)
        
    def move_to(self, positions, duration=0.5):
        """Quick juggling movements"""
        steps = int(duration * 50)
        for step in range(steps):
            for joint, pos in positions.items():
                if joint in self.pubs:
                    self.pubs[joint].publish(Float64(pos))
                elif joint == "Gripper":
                    self.gripper_pub.publish(Float64(pos))
            time.sleep(0.02)
    
    def juggle_pattern(self):
        """Simulate 3-ball juggling pattern"""
        rospy.loginfo("🤹 Starting juggling routine...")
        
        # Juggling positions: catch low, throw high
        positions = [
            # Catch left, throw right
            {"Joint01": -0.3, "Joint02": -0.6, "Joint03": 1.2, "Joint04": 0.0, "Joint05": -0.6, "Joint06": 0.0, "Gripper": 0.6},
            {"Joint01": 0.3, "Joint02": -0.2, "Joint03": 0.8, "Joint04": 0.0, "Joint05": -0.6, "Joint06": 0.0, "Gripper": 0.0},
            
            # Catch right, throw left  
            {"Joint01": 0.3, "Joint02": -0.6, "Joint03": 1.2, "Joint04": 0.0, "Joint05": -0.6, "Joint06": 0.0, "Gripper": 0.6},
            {"Joint01": -0.3, "Joint02": -0.2, "Joint03": 0.8, "Joint04": 0.0, "Joint05": -0.6, "Joint06": 0.0, "Gripper": 0.0},
            
            # Center catch and throw
            {"Joint01": 0.0, "Joint02": -0.6, "Joint03": 1.2, "Joint04": 0.0, "Joint05": -0.6, "Joint06": 0.0, "Gripper": 0.6},
            {"Joint01": 0.0, "Joint02": -0.2, "Joint03": 0.8, "Joint04": 0.0, "Joint05": -0.6, "Joint06": 0.0, "Gripper": 0.0},
        ]
        
        # Perform juggling cycles
        for cycle in range(10):
            for pos in positions:
                self.move_to(pos, 0.3)
    
    def finale_throw(self):
        """Dramatic finale with high throws"""
        rospy.loginfo("🎪 Grand finale!")
        
        # Multiple quick throws
        for i in range(5):
            # Low catch
            catch_pos = {
                "Joint01": 0.2 * math.sin(i),
                "Joint02": -0.8,
                "Joint03": 1.5,
                "Joint04": 0.0,
                "Joint05": -0.7,
                "Joint06": 0.0,
                "Gripper": 0.6
            }
            self.move_to(catch_pos, 0.2)
            
            # High throw
            throw_pos = {
                "Joint01": 0.2 * math.sin(i),
                "Joint02": 0.0,
                "Joint03": 0.3,
                "Joint04": 0.0,
                "Joint05": -0.3,
                "Joint06": 0.0,
                "Gripper": 0.0
            }
            self.move_to(throw_pos, 0.3)
    
    def run(self):
        rospy.loginfo("🤹‍♂️ Z1 Juggler ready to perform!")
        
        # Warm up
        rospy.loginfo("🔥 Warming up...")
        warm_up = {"Joint01": 0.0, "Joint02": -0.4, "Joint03": 1.0, "Joint04": 0.0, "Joint05": -0.6, "Joint06": 0.0, "Gripper": 0.3}
        self.move_to(warm_up, 2.0)
        
        # Main juggling routine
        self.juggle_pattern()
        
        # Finale
        self.finale_throw()
        
        # Bow
        bow = {"Joint01": 0.0, "Joint02": -0.8, "Joint03": 1.5, "Joint04": 0.0, "Joint05": -0.7, "Joint06": 0.0, "Gripper": 0.0}
        self.move_to(bow, 2.0)
        time.sleep(2)
        
        # Return home
        home = {f"Joint0{i}": 0.0 for i in range(1, 7)}
        home["Gripper"] = 0.0
        self.move_to(home, 3.0)
        
        rospy.loginfo("👏 Juggling performance complete!")

if __name__ == "__main__":
    try:
        juggler = Z1Juggler()
        juggler.run()
    except rospy.ROSInterruptException:
        pass