#!/usr/bin/env python3

"""
EMERGENCY STOP for Real Unitree Z1 Hardware
⚠️  Use this to immediately stop all robot motion
⚠️  Keep this script easily accessible during real robot operation
"""

import rospy
import sys
from unitree_legged_msgs.msg import MotorCmd

class EmergencyStopReal:
    def __init__(self):
        rospy.init_node('z1_emergency_stop_real', anonymous=True)
        
        self.joints = ['Joint01', 'Joint02', 'Joint03', 'Joint04', 'Joint05', 'Joint06']
        self.publishers = {}
        
        # Setup publishers for real hardware
        for joint in self.joints:
            self.publishers[joint] = rospy.Publisher(
                f'/z1_real/{joint}_controller/command', 
                MotorCmd, queue_size=1
            )
            
        rospy.logwarn("Emergency Stop Ready - Press Enter to STOP ALL MOTORS")
        
    def emergency_stop(self):
        """Immediately disable all motors"""
        rospy.logerr("🚨 EMERGENCY STOP ACTIVATED! 🚨")
        
        for joint in self.joints:
            cmd = MotorCmd()
            cmd.mode = 0      # Disable mode
            cmd.q = 0.0
            cmd.dq = 0.0  
            cmd.tau = 0.0
            cmd.Kp = 0.0
            cmd.Kd = 0.0
            
            # Send stop command multiple times for reliability
            for _ in range(10):
                self.publishers[joint].publish(cmd)
                rospy.sleep(0.01)
                
        rospy.logerr("ALL MOTORS DISABLED")
        rospy.logerr("Robot should be stopped - verify visually before approaching")

if __name__ == '__main__':
    try:
        estop = EmergencyStopReal()
        
        # Wait for user input
        input("Press Enter to activate EMERGENCY STOP...")
        
        estop.emergency_stop()
        
    except Exception as e:
        rospy.logerr(f"Emergency stop error: {e}")
        
    finally:
        rospy.logwarn("Emergency stop script completed")
        sys.exit(0)