#!/usr/bin/env python3

"""
Z1 Bartender - Cocktail mixing choreography with realistic movements
"""

import rospy
import time
import math
from unitree_legged_msgs.msg import MotorCmd

class Z1Bartender:
    def __init__(self):
        rospy.init_node("z1_bartender")
        
        # Publishers for each joint
        self.pubs = {}
        joints = ["Joint01", "Joint02", "Joint03", "Joint04", "Joint05", "Joint06", "Gripper"]
        
        for joint in joints:
            controller = f"{joint}_controller" if joint != "Gripper" else "Gripper_controller"
            topic = f"/z1_gazebo/{controller}/command"
            self.pubs[joint] = rospy.Publisher(topic, MotorCmd, queue_size=1)
        
        # Current positions
        self.positions = {j: 0.0 for j in joints}
        
        rospy.loginfo("🍸 Z1 Bartender initialized - Ready to mix cocktails!")
    
    def move_to_position(self, joint_positions, duration=2.0):
        """Smoothly move to target positions over duration"""
        start_positions = self.positions.copy()
        steps = int(duration * 50)  # 50Hz
        
        for step in range(steps + 1):
            progress = step / steps
            
            for joint, target in joint_positions.items():
                if joint in self.positions:
                    start = start_positions[joint]
                    current = start + (target - start) * progress
                    
                    msg = MotorCmd()
                    msg.mode = 10
                    msg.q = current
                    msg.Kp = 35.0
                    msg.Kd = 1.5
                    
                    self.pubs[joint].publish(msg)
                    self.positions[joint] = current
            
            time.sleep(0.02)  # 50Hz
    
    def shake_motion(self, intensity=0.3, duration=3.0):
        """Cocktail shaking motion"""
        rospy.loginfo("🥤 Shaking cocktail...")
        
        base_pos = self.positions.copy()
        steps = int(duration * 10)  # 10Hz for shaking
        
        for i in range(steps):
            # Rhythmic shaking motion
            shake_offset = intensity * math.sin(i * 0.8) * (1.0 - i/steps)  # Fade out
            
            positions = {
                "Joint01": base_pos["Joint01"] + shake_offset * 0.5,
                "Joint02": base_pos["Joint02"] + shake_offset * 0.3,
                "Joint04": base_pos["Joint04"] + shake_offset * 0.8,
                "Joint05": base_pos["Joint05"] + shake_offset * 0.4
            }
            
            self.move_to_position(positions, 0.1)
    
    def pour_motion(self, pour_angle=0.8):
        """Pouring motion"""
        rospy.loginfo("🍷 Pouring drink...")
        
        # Tilt for pouring
        pour_positions = {
            "Joint05": pour_angle,  # Wrist pitch for pouring
            "Joint06": 0.2         # Slight wrist roll
        }
        self.move_to_position(pour_positions, 1.5)
        
        # Hold pour position
        time.sleep(2.0)
        
        # Return to upright
        upright_positions = {
            "Joint05": 0.0,
            "Joint06": 0.0
        }
        self.move_to_position(upright_positions, 1.0)
    
    def garnish_motion(self):
        """Delicate garnish placement"""
        rospy.loginfo("🍋 Adding garnish...")
        
        # Precise movements for garnish
        garnish_positions = {
            "Joint02": -0.3,  # Lower shoulder
            "Joint03": 1.2,   # Extend elbow
            "Joint05": -0.4,  # Precise wrist angle
            "Gripper": 0.2    # Gentle grip
        }
        self.move_to_position(garnish_positions, 2.0)
        
        # Place garnish with small movements
        for i in range(3):
            self.move_to_position({"Joint05": -0.4 + 0.1 * math.sin(i)}, 0.5)
        
        # Release garnish
        self.move_to_position({"Gripper": 0.5}, 0.5)
    
    def flourish_motion(self):
        """Bartender flourish"""
        rospy.loginfo("✨ Bartender flourish!")
        
        # Dramatic arm sweep
        flourish_positions = {
            "Joint01": 0.8,   # Wide base rotation
            "Joint02": -0.2,  # Shoulder position
            "Joint03": 1.5,   # Extended elbow
            "Joint04": 0.0,   # Forearm
            "Joint05": -0.6,  # Wrist down
            "Joint06": 0.4    # Wrist roll
        }
        self.move_to_position(flourish_positions, 1.5)
        
        # Quick wrist flicks
        for i in range(4):
            self.move_to_position({"Joint06": 0.4 + 0.3 * ((-1) ** i)}, 0.3)
    
    def bartender_sequence(self):
        """Complete bartender performance"""
        rospy.loginfo("🎭 Starting bartender performance...")
        
        # 1. Starting position - ready to work
        rospy.loginfo("📍 Getting ready...")
        ready_pos = {
            "Joint01": 0.0, "Joint02": -0.5, "Joint03": 1.0,
            "Joint04": 0.0, "Joint05": -0.3, "Joint06": 0.0, "Gripper": 0.0
        }
        self.move_to_position(ready_pos, 3.0)
        time.sleep(1.0)
        
        # 2. Reach for bottle
        rospy.loginfo("🍾 Reaching for bottle...")
        bottle_pos = {
            "Joint01": -0.6, "Joint02": -0.3, "Joint03": 1.4,
            "Joint04": 0.2, "Joint05": -0.5, "Gripper": 0.0
        }
        self.move_to_position(bottle_pos, 2.0)
        
        # 3. Grab bottle
        rospy.loginfo("🤏 Grabbing bottle...")
        self.move_to_position({"Gripper": 0.4}, 1.0)
        time.sleep(0.5)
        
        # 4. Lift and position for pouring
        rospy.loginfo("⬆️ Lifting bottle...")
        lift_pos = {
            "Joint01": 0.3, "Joint02": -0.4, "Joint03": 1.2,
            "Joint04": -0.2, "Joint05": 0.0
        }
        self.move_to_position(lift_pos, 2.0)
        
        # 5. Pour into glass
        self.pour_motion()
        
        # 6. Set bottle down
        rospy.loginfo("📍 Setting bottle down...")
        self.move_to_position(bottle_pos, 1.5)
        self.move_to_position({"Gripper": 0.0}, 0.5)
        
        # 7. Get shaker
        rospy.loginfo("🥤 Getting shaker...")
        shaker_pos = {
            "Joint01": 0.4, "Joint02": -0.2, "Joint03": 1.3,
            "Joint04": 0.0, "Joint05": -0.2
        }
        self.move_to_position(shaker_pos, 2.0)
        self.move_to_position({"Gripper": 0.5}, 0.5)
        
        # 8. Shake cocktail
        self.shake_motion(intensity=0.4, duration=4.0)
        
        # 9. Pour from shaker
        rospy.loginfo("🍸 Pouring cocktail...")
        pour_pos = {
            "Joint01": 0.0, "Joint02": -0.3, "Joint03": 1.1,
            "Joint04": 0.1, "Joint05": 0.0
        }
        self.move_to_position(pour_pos, 1.5)
        self.pour_motion(pour_angle=0.9)
        
        # 10. Add garnish
        self.move_to_position({"Gripper": 0.0}, 0.5)
        self.garnish_motion()
        
        # 11. Final flourish
        self.flourish_motion()
        
        # 12. Present drink
        rospy.loginfo("🎉 Presenting cocktail!")
        present_pos = {
            "Joint01": 0.0, "Joint02": -0.4, "Joint03": 0.8,
            "Joint04": 0.0, "Joint05": -0.2, "Joint06": 0.0, "Gripper": 0.0
        }
        self.move_to_position(present_pos, 2.0)
        
        # Bow
        rospy.loginfo("🙇 Taking a bow...")
        bow_pos = {"Joint02": -0.8, "Joint03": 1.5}
        self.move_to_position(bow_pos, 1.5)
        time.sleep(1.0)
        
        # Return home
        rospy.loginfo("🏠 Returning to home position...")
        home_pos = {j: 0.0 for j in self.positions.keys()}
        self.move_to_position(home_pos, 3.0)
        
        rospy.loginfo("🍸✨ Cocktail service complete! Enjoy your drink! 🍸✨")
    
    def run(self):
        """Run the bartender performance"""
        rospy.loginfo("🍸 Welcome to Z1's Cocktail Bar!")
        rospy.loginfo("🎭 Preparing to mix your drink...")
        time.sleep(2.0)
        
        try:
            self.bartender_sequence()
        except rospy.ROSInterruptException:
            rospy.loginfo("🛑 Bartender service interrupted")
        except Exception as e:
            rospy.logerr(f"❌ Bartender error: {e}")

if __name__ == "__main__":
    try:
        bartender = Z1Bartender()
        bartender.run()
    except KeyboardInterrupt:
        rospy.loginfo("🛑 Bartender stopped by user")
    except Exception as e:
        rospy.logerr(f"❌ Failed to start bartender: {e}")