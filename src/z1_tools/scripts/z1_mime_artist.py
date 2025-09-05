#!/usr/bin/env python3

"""
Z1 Mime Artist - Silent performance art
"""

import rospy
import math
import time
from std_msgs.msg import Float64

class Z1MimeArtist:
    def __init__(self):
        rospy.init_node("z1_mime_artist")
        
        self.pubs = {}
        for i in range(1, 7):
            topic = f"/z1_gazebo/Joint0{i}_controller/command"
            self.pubs[f"Joint0{i}"] = rospy.Publisher(topic, Float64, queue_size=1)
        
        self.gripper_pub = rospy.Publisher("/z1_gazebo/Gripper_controller/command", Float64, queue_size=1)
        
    def move_to(self, positions, duration=2.0):
        """Precise, deliberate mime movements"""
        steps = int(duration * 50)
        for step in range(steps):
            for joint, pos in positions.items():
                if joint in self.pubs:
                    self.pubs[joint].publish(Float64(pos))
                elif joint == "Gripper":
                    self.gripper_pub.publish(Float64(pos))
            time.sleep(0.02)
    
    def invisible_box(self):
        """Classic invisible box routine"""
        rospy.loginfo("📦 Trapped in an invisible box...")
        
        # Feel the walls
        walls = [
            # Front wall
            {"Joint01": 0.0, "Joint02": -0.2, "Joint03": 0.8, "Joint04": 0.0, "Joint05": -0.6, "Joint06": 0.0, "Gripper": 0.0},
            # Right wall  
            {"Joint01": 0.4, "Joint02": -0.2, "Joint03": 0.8, "Joint04": 0.0, "Joint05": -0.6, "Joint06": 0.0, "Gripper": 0.0},
            # Back wall
            {"Joint01": 0.0, "Joint02": 0.2, "Joint03": 0.3, "Joint04": 0.0, "Joint05": -0.3, "Joint06": 0.0, "Gripper": 0.0},
            # Left wall
            {"Joint01": -0.4, "Joint02": -0.2, "Joint03": 0.8, "Joint04": 0.0, "Joint05": -0.6, "Joint06": 0.0, "Gripper": 0.0},
            # Top
            {"Joint01": 0.0, "Joint02": 0.3, "Joint03": 0.2, "Joint04": 0.0, "Joint05": -0.5, "Joint06": 0.0, "Gripper": 0.0},
            # Bottom
            {"Joint01": 0.0, "Joint02": -0.8, "Joint03": 1.5, "Joint04": 0.0, "Joint05": -0.7, "Joint06": 0.0, "Gripper": 0.0},
        ]
        
        for wall in walls:
            self.move_to(wall, 2.0)
            time.sleep(1.0)  # Pause to "feel" the wall
        
        # Panic - try to escape
        for i in range(5):
            escape_attempt = {
                "Joint01": 0.3 * math.sin(i),
                "Joint02": -0.2 + 0.2 * math.cos(i),
                "Joint03": 0.8 + 0.2 * math.sin(i * 2),
                "Joint04": 0.2 * math.cos(i * 3),
                "Joint05": -0.6,
                "Joint06": 0.1 * math.sin(i * 4),
                "Gripper": 0.3
            }
            self.move_to(escape_attempt, 0.5)
    
    def invisible_rope(self):
        """Climbing an invisible rope"""
        rospy.loginfo("🪢 Climbing the invisible rope...")
        
        # Reach up and grab rope
        grab_rope = {"Joint01": 0.0, "Joint02": 0.4, "Joint03": 0.0, "Joint04": 0.0, "Joint05": -0.4, "Joint06": 0.0, "Gripper": 0.6}
        self.move_to(grab_rope, 2.0)
        time.sleep(1.0)
        
        # Climbing motion - alternating hands
        for climb in range(6):
            # Pull up with current hand, reach higher with other
            if climb % 2 == 0:
                # Right hand pulls, left reaches up
                pull_pos = {"Joint01": 0.1, "Joint02": 0.2, "Joint03": 0.3, "Joint04": 0.0, "Joint05": -0.5, "Joint06": 0.0, "Gripper": 0.6}
                self.move_to(pull_pos, 1.0)
                
                reach_pos = {"Joint01": -0.1, "Joint02": 0.5, "Joint03": 0.0, "Joint04": 0.0, "Joint05": -0.5, "Joint06": 0.0, "Gripper": 0.6}
                self.move_to(reach_pos, 1.0)
            else:
                # Left hand pulls, right reaches up
                pull_pos = {"Joint01": -0.1, "Joint02": 0.2, "Joint03": 0.3, "Joint04": 0.0, "Joint05": -0.5, "Joint06": 0.0, "Gripper": 0.6}
                self.move_to(pull_pos, 1.0)
                
                reach_pos = {"Joint01": 0.1, "Joint02": 0.5, "Joint03": 0.0, "Joint04": 0.0, "Joint05": -0.5, "Joint06": 0.0, "Gripper": 0.6}
                self.move_to(reach_pos, 1.0)
        
        # Look down (afraid of height)
        look_down = {"Joint01": 0.0, "Joint02": -0.3, "Joint03": 0.8, "Joint04": 0.0, "Joint05": -0.5, "Joint06": 0.0, "Gripper": 0.3}
        self.move_to(look_down, 2.0)
        time.sleep(2.0)
    
    def invisible_wall(self):
        """Walking into invisible wall"""
        rospy.loginfo("🧱 Walking into an invisible wall...")
        
        # Normal walking motion
        walk_pos = {"Joint01": 0.0, "Joint02": -0.3, "Joint03": 1.0, "Joint04": 0.0, "Joint05": -0.7, "Joint06": 0.0, "Gripper": 0.3}
        self.move_to(walk_pos, 2.0)
        
        # Sudden stop - hit the wall
        hit_wall = {"Joint01": 0.0, "Joint02": -0.1, "Joint03": 0.6, "Joint04": 0.0, "Joint05": -0.5, "Joint06": 0.0, "Gripper": 0.0}
        self.move_to(hit_wall, 0.3)  # Quick stop
        
        # Recoil in surprise
        recoil = {"Joint01": 0.0, "Joint02": 0.1, "Joint03": 0.4, "Joint04": 0.0, "Joint05": -0.4, "Joint06": 0.0, "Gripper": 0.6}
        self.move_to(recoil, 0.5)
        time.sleep(1.0)
        
        # Feel the wall with hands
        feel_wall = {"Joint01": 0.0, "Joint02": -0.2, "Joint03": 0.8, "Joint04": 0.0, "Joint05": -0.6, "Joint06": 0.0, "Gripper": 0.0}
        self.move_to(feel_wall, 2.0)
        
        # Try to push through
        push = {"Joint01": 0.0, "Joint02": -0.4, "Joint03": 1.0, "Joint04": 0.0, "Joint05": -0.6, "Joint06": 0.0, "Gripper": 0.0}
        self.move_to(push, 1.5)
        time.sleep(1.0)
        
        # Give up, shrug
        shrug = {"Joint01": 0.0, "Joint02": 0.2, "Joint03": 0.5, "Joint04": 0.0, "Joint05": -0.3, "Joint06": 0.0, "Gripper": 0.3}
        self.move_to(shrug, 2.0)
    
    def invisible_ball(self):
        """Juggling invisible balls"""
        rospy.loginfo("⚽ Juggling invisible balls...")
        
        # Toss and catch pattern
        for toss in range(8):
            # Catch low
            catch_pos = {
                "Joint01": 0.2 * math.sin(toss * 0.5),
                "Joint02": -0.5,
                "Joint03": 1.2,
                "Joint04": 0.0,
                "Joint05": -0.7,
                "Joint06": 0.0,
                "Gripper": 0.6
            }
            self.move_to(catch_pos, 0.3)
            
            # Toss high
            toss_pos = {
                "Joint01": 0.2 * math.sin(toss * 0.5),
                "Joint02": 0.0,
                "Joint03": 0.5,
                "Joint04": 0.0,
                "Joint05": -0.5,
                "Joint06": 0.0,
                "Gripper": 0.0
            }
            self.move_to(toss_pos, 0.4)
            
            # Follow ball with eyes (head movement)
            follow = {
                "Joint01": 0.2 * math.sin(toss * 0.5),
                "Joint02": 0.2,
                "Joint03": 0.3,
                "Joint04": 0.0,
                "Joint05": -0.3,
                "Joint06": 0.1 * math.sin(toss),
                "Gripper": 0.3
            }
            self.move_to(follow, 0.3)
    
    def trapped_in_shrinking_room(self):
        """Room is getting smaller"""
        rospy.loginfo("📐 The room is shrinking!")
        
        # Start normal
        normal = {"Joint01": 0.0, "Joint02": -0.2, "Joint03": 0.8, "Joint04": 0.0, "Joint05": -0.6, "Joint06": 0.0, "Gripper": 0.3}
        self.move_to(normal, 2.0)
        
        # Notice ceiling getting lower
        duck_slightly = {"Joint01": 0.0, "Joint02": -0.3, "Joint03": 1.0, "Joint04": 0.0, "Joint05": -0.7, "Joint06": 0.0, "Gripper": 0.3}
        self.move_to(duck_slightly, 2.0)
        time.sleep(1.0)
        
        # Duck more
        duck_more = {"Joint01": 0.0, "Joint02": -0.5, "Joint03": 1.3, "Joint04": 0.0, "Joint05": -0.8, "Joint06": 0.0, "Gripper": 0.3}
        self.move_to(duck_more, 2.0)
        time.sleep(1.0)
        
        # Crouch down
        crouch = {"Joint01": 0.0, "Joint02": -0.8, "Joint03": 1.6, "Joint04": 0.0, "Joint05": -0.8, "Joint06": 0.0, "Gripper": 0.0}
        self.move_to(crouch, 2.0)
        time.sleep(1.0)
        
        # Panic - try to hold up ceiling
        hold_ceiling = {"Joint01": 0.0, "Joint02": 0.3, "Joint03": 0.2, "Joint04": 0.0, "Joint05": -0.5, "Joint06": 0.0, "Gripper": 0.0}
        self.move_to(hold_ceiling, 1.0)
        time.sleep(2.0)
    
    def run(self):
        rospy.loginfo("🎭 Z1 Mime Artist ready to perform!")
        rospy.loginfo("🤫 *Silent performance begins*")
        
        # Performance sequence
        self.invisible_box()
        time.sleep(2)
        
        self.invisible_wall()
        time.sleep(2)
        
        self.invisible_rope()
        time.sleep(2)
        
        self.invisible_ball()
        time.sleep(2)
        
        self.trapped_in_shrinking_room()
        time.sleep(2)
        
        # Final bow
        bow = {"Joint01": 0.0, "Joint02": -0.6, "Joint03": 1.3, "Joint04": 0.0, "Joint05": -0.7, "Joint06": 0.0, "Gripper": 0.0}
        self.move_to(bow, 3.0)
        time.sleep(3.0)
        
        # Return to neutral
        neutral = {"Joint01": 0.0, "Joint02": 0.0, "Joint03": 0.0, "Joint04": 0.0, "Joint05": 0.0, "Joint06": 0.0, "Gripper": 0.0}
        self.move_to(neutral, 3.0)
        
        rospy.loginfo("👏 *Applause* Performance complete!")

if __name__ == "__main__":
    try:
        mime = Z1MimeArtist()
        mime.run()
    except rospy.ROSInterruptException:
        pass