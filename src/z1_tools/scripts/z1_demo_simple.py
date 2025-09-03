#!/usr/bin/env python3

"""
Simple Z1 Demo - Basic pick and place demonstration
Shows smooth movement between predefined safe positions
"""

import rospy
import time
from unitree_legged_msgs.msg import MotorCmd

class SimpleZ1Demo:
    def __init__(self):
        rospy.init_node("z1_demo_simple")
        
        self.joints = ["Joint01", "Joint02", "Joint03", "Joint04", "Joint05", "Joint06", "Gripper"]
        
        # Publishers
        self.pubs = {}
        for joint in self.joints:
            topic = f"/z1_gazebo/{joint}_controller/command"
            self.pubs[joint] = rospy.Publisher(topic, MotorCmd, queue_size=1)
        
        # Predefined safe positions
        self.positions = {
            "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "ready": [0.0, -0.5, 1.0, 0.0, -0.5, 0.0, 0.0],
            "pick": [0.5, -0.3, 1.2, 0.0, -0.9, 0.0, 0.0],
            "lift": [0.5, -0.5, 1.0, 0.0, -0.5, 0.0, 0.6],
            "place": [-0.5, -0.3, 1.2, 0.0, -0.9, 0.0, 0.6],
            "drop": [-0.5, -0.3, 1.2, 0.0, -0.9, 0.0, 0.0]
        }
        
    def send_position(self, joint_positions, duration=3.0):
        """Move to target positions smoothly"""
        rospy.loginfo(f"Moving to position over {duration}s")
        
        start_time = time.time()
        rate = rospy.Rate(50)  # 50Hz
        
        while time.time() - start_time < duration and not rospy.is_shutdown():
            for i, joint in enumerate(self.joints):
                msg = MotorCmd()
                msg.mode = 10
                msg.q = float(joint_positions[i])
                msg.Kp = 25.0  # Slightly lower for smoother movement
                msg.Kd = 1.0
                
                self.pubs[joint].publish(msg)
            
            rate.sleep()
        
        rospy.loginfo("Position reached")
    
    def run_demo(self):
        """Run the complete pick and place demo"""
        rospy.loginfo("Starting Z1 Simple Demo")
        
        sequence = [
            ("home", "Starting at home position", 2.0),
            ("ready", "Moving to ready position", 3.0),
            ("pick", "Moving to pick position", 3.0),
            ("pick", "Closing gripper", 1.0),  # Close gripper
            ("lift", "Lifting object", 2.0),
            ("place", "Moving to place position", 4.0),
            ("drop", "Opening gripper", 1.0),  # Open gripper
            ("ready", "Returning to ready", 3.0),
            ("home", "Returning home", 3.0)
        ]
        
        for pos_name, description, duration in sequence:
            rospy.loginfo(description)
            
            # Special handling for gripper actions
            if "gripper" in description.lower():
                positions = list(self.positions[pos_name])
                if "closing" in description.lower():
                    positions[6] = 0.6  # Close gripper
                else:
                    positions[6] = 0.0  # Open gripper
            else:
                positions = self.positions[pos_name]
            
            self.send_position(positions, duration)
            time.sleep(0.5)  # Brief pause between movements
        
        rospy.loginfo("Demo completed!")

if __name__ == "__main__":
    try:
        demo = SimpleZ1Demo()
        demo.run_demo()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Demo interrupted by user")
    except Exception as e:
        rospy.logerr(f"Demo error: {e}")