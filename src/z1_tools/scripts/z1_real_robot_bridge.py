#!/usr/bin/env python3

"""
Z1 Real Robot Bridge - Connects simulation controls to real robot
This script bridges ROS simulation topics to real robot SDK calls
"""

import rospy
import sys
import os
from std_msgs.msg import Float64

# Add z1_sdk to path
sys.path.append(os.path.join(os.path.dirname(__file__), '../../z1_sdk/examples_py'))

try:
    import unitree_arm_interface
    REAL_ROBOT_AVAILABLE = True
except ImportError:
    REAL_ROBOT_AVAILABLE = False
    rospy.logwarn("Real robot SDK not available - simulation only")

class Z1RealRobotBridge:
    def __init__(self):
        rospy.init_node("z1_real_robot_bridge")
        
        if not REAL_ROBOT_AVAILABLE:
            rospy.logerr("Real robot SDK not available. Install z1_sdk first.")
            sys.exit(1)
        
        # Initialize real robot connection
        try:
            self.arm = unitree_arm_interface.ArmInterface()
            rospy.loginfo("Connected to real Z1 robot")
        except Exception as e:
            rospy.logerr(f"Failed to connect to real robot: {e}")
            sys.exit(1)
        
        # Joint mapping
        self.joint_map = {
            "Joint01": 0,
            "Joint02": 1,
            "Joint03": 2,
            "Joint04": 3,
            "Joint05": 4,
            "Joint06": 5,
            "Gripper": 6
        }
        
        # Current joint positions
        self.joint_positions = [0.0] * 7
        
        # Subscribe to simulation control topics
        self.subscribers = {}
        for joint in self.joint_map.keys():
            if joint == "Gripper":
                controller = "Gripper_controller"
            else:
                controller = f"{joint}_controller"
            
            topic = f"/z1_gazebo/{controller}/command"
            self.subscribers[joint] = rospy.Subscriber(
                topic, Float64, 
                lambda msg, j=joint: self.joint_command_callback(msg, j)
            )
        
        rospy.loginfo("Z1 Real Robot Bridge initialized")
        rospy.logwarn("SAFETY: Ensure robot is in safe position and emergency stop is accessible")
        
    def joint_command_callback(self, msg, joint_name):
        """Convert simulation commands to real robot commands"""
        joint_idx = self.joint_map[joint_name]
        
        # Convert effort command to position (simple integration)
        # In real implementation, you'd use proper control algorithms
        effort = msg.data
        position_delta = effort * 0.001  # Scale factor
        
        self.joint_positions[joint_idx] += position_delta
        
        # Apply safety limits
        limits = {
            0: (-1.2, 1.2),   # Joint01
            1: (-1.0, 1.0),   # Joint02  
            2: (0.0, 2.4),    # Joint03
            3: (-1.2, 1.2),   # Joint04
            4: (-1.0, 1.0),   # Joint05
            5: (-1.2, 1.2),   # Joint06
            6: (0.0, 0.6)     # Gripper
        }
        
        min_pos, max_pos = limits[joint_idx]
        self.joint_positions[joint_idx] = max(min_pos, min(max_pos, self.joint_positions[joint_idx]))
        
        # Send to real robot
        try:
            self.arm.setJointPosition(joint_idx, self.joint_positions[joint_idx])
        except Exception as e:
            rospy.logerr(f"Failed to send command to real robot: {e}")
    
    def run(self):
        """Main loop"""
        rate = rospy.Rate(50)  # 50Hz control loop
        
        while not rospy.is_shutdown():
            try:
                # Send current positions to real robot
                self.arm.sendCommand()
                rate.sleep()
            except Exception as e:
                rospy.logerr(f"Real robot communication error: {e}")
                break
        
        # Emergency stop on shutdown
        try:
            for i in range(7):
                self.arm.setJointPosition(i, 0.0)
            self.arm.sendCommand()
        except:
            pass

if __name__ == "__main__":
    try:
        bridge = Z1RealRobotBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Bridge shutdown requested")
    except Exception as e:
        rospy.logerr(f"Bridge error: {e}")