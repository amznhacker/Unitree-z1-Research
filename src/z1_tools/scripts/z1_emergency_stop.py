#!/usr/bin/env python3
"""
Z1 Emergency Stop Script
Immediately stops all motion and returns arm to safe neutral position
"""
import rospy, signal, sys, math
from unitree_legged_msgs.msg import MotorCmd

class Z1EmergencyStop:
    def __init__(self):
        rospy.init_node("z1_emergency_stop", anonymous=True)
        
        # Publishers for all joints
        self.pubs = {}
        joints = ["Joint01", "Joint02", "Joint03", "Joint04", "Joint05", "Joint06", "Gripper"]
        
        for joint in joints:
            topic = f"/z1_gazebo/{joint}_controller/command"
            self.pubs[joint] = rospy.Publisher(topic, MotorCmd, queue_size=10)
        
        self.rate = rospy.Rate(100)  # High frequency for emergency
        
        # Setup signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)
    
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully"""
        rospy.logwarn("Emergency stop signal received!")
        self.emergency_stop()
        sys.exit(0)
    
    def emergency_stop(self):
        """Execute emergency stop procedure"""
        rospy.logwarn("EMERGENCY STOP ACTIVATED!")
        rospy.logwarn("Stopping all motion and returning to safe position...")
        
        # Send zero velocity commands first (immediate stop)
        for _ in range(10):  # 0.1 seconds of zero velocity
            for joint in self.pubs.keys():
                msg = MotorCmd()
                msg.mode = 0  # Velocity mode for immediate stop
                msg.q = 0.0
                msg.dq = 0.0  # Zero velocity
                msg.tau = 0.0
                msg.Kp = 0.0
                msg.Kd = 5.0  # High damping
                self.pubs[joint].publish(msg)
            self.rate.sleep()
        
        # Gradually move to neutral position with high damping
        rospy.logwarn("Moving to neutral position...")
        
        duration = 3.0  # 3 seconds to reach neutral
        steps = int(duration * 100)  # 100 Hz
        
        for step in range(steps):
            if rospy.is_shutdown():
                break
            
            # Gradual approach to neutral
            alpha = step / float(steps)
            smooth_alpha = 0.5 * (1 - math.cos(math.pi * alpha))
            
            for joint in self.pubs.keys():
                msg = MotorCmd()
                msg.mode = 10  # Position mode
                msg.q = 0.0 * smooth_alpha  # Target neutral (0.0)
                msg.dq = 0.0
                msg.tau = 0.0
                msg.Kp = 20.0  # Lower stiffness for safety
                msg.Kd = 3.0   # Higher damping for stability
                self.pubs[joint].publish(msg)
            
            self.rate.sleep()
        
        # Hold neutral position
        rospy.logwarn("Holding neutral position...")
        for _ in range(200):  # 2 seconds
            for joint in self.pubs.keys():
                msg = MotorCmd()
                msg.mode = 10
                msg.q = 0.0
                msg.dq = 0.0
                msg.tau = 0.0
                msg.Kp = 15.0  # Gentle hold
                msg.Kd = 2.0
                self.pubs[joint].publish(msg)
            self.rate.sleep()
        
        rospy.logwarn("Emergency stop completed. Arm is in safe neutral position.")
    
    def monitor_mode(self):
        """Continuous monitoring mode - press Ctrl+C to activate emergency stop"""
        rospy.loginfo("Emergency stop monitor active.")
        rospy.loginfo("Press Ctrl+C at any time to activate emergency stop.")
        
        try:
            while not rospy.is_shutdown():
                rospy.sleep(0.1)
        except rospy.ROSInterruptException:
            pass

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Z1 Emergency Stop")
    parser.add_argument("--monitor", action="store_true", 
                       help="Run in monitor mode (press Ctrl+C to stop)")
    args = parser.parse_args()
    
    try:
        estop = Z1EmergencyStop()
        
        if args.monitor:
            estop.monitor_mode()
        else:
            # Immediate emergency stop
            estop.emergency_stop()
    
    except Exception as e:
        rospy.logerr(f"Emergency stop failed: {e}")
        # Last resort - try to send zero commands
        try:
            import time
            pubs = {}
            joints = ["Joint01", "Joint02", "Joint03", "Joint04", "Joint05", "Joint06", "Gripper"]
            
            for joint in joints:
                topic = f"/z1_gazebo/{joint}_controller/command"
                pubs[joint] = rospy.Publisher(topic, MotorCmd, queue_size=10)
            
            for _ in range(50):
                for joint in joints:
                    msg = MotorCmd()
                    msg.mode = 0
                    msg.q = 0.0
                    msg.dq = 0.0
                    msg.tau = 0.0
                    msg.Kp = 0.0
                    msg.Kd = 10.0
                    pubs[joint].publish(msg)
                time.sleep(0.02)
        except:
            rospy.logerr("Critical failure in emergency stop!")

if __name__ == "__main__":
    main()