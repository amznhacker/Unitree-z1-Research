#!/usr/bin/env python3
"""
Z1 Safe Limits Tester
Test all joints within safe operating limits
"""
import rospy, math, time, argparse
from unitree_legged_msgs.msg import MotorCmd

class Z1SafeLimits:
    def __init__(self, mode=10, kp=35.0, kd=1.5):
        rospy.init_node("z1_safe_limits")
        
        self.mode = mode
        self.kp = kp
        self.kd = kd
        
        # Conservative safe limits (radians)
        self.limits = {
            "Joint01": (-1.2, 1.2),    # Base yaw ±69°
            "Joint02": (-1.0, 1.0),    # Shoulder pitch ±57°
            "Joint03": (0.0, 2.4),     # Elbow 0° to 137°
            "Joint04": (-1.2, 1.2),    # Forearm roll ±69°
            "Joint05": (-1.0, 1.0),    # Wrist pitch ±57°
            "Joint06": (-1.2, 1.2),    # Wrist roll ±69°
            "Gripper": (0.0, 0.6)      # Gripper 0-60% open
        }
        
        # Publishers
        self.pubs = {}
        for joint in self.limits.keys():
            topic = f"/z1_gazebo/{joint}_controller/command"
            self.pubs[joint] = rospy.Publisher(topic, MotorCmd, queue_size=10)
        
        self.rate = rospy.Rate(50)
    
    def send_command(self, joint, position):
        """Send motor command"""
        msg = MotorCmd()
        msg.mode = self.mode
        msg.q = float(position)
        msg.dq = 0.0
        msg.tau = 0.0
        msg.Kp = self.kp
        msg.Kd = self.kd
        self.pubs[joint].publish(msg)
    
    def move_to_position(self, joint, target, duration=2.0):
        """Smoothly move joint to target position"""
        rospy.loginfo(f"Moving {joint} to {target:.2f} rad ({math.degrees(target):.1f}°)")
        
        start_time = time.time()
        
        while not rospy.is_shutdown():
            elapsed = time.time() - start_time
            if elapsed >= duration:
                break
            
            # Smooth interpolation
            alpha = elapsed / duration
            alpha = min(1.0, alpha)
            smooth_alpha = 0.5 * (1 - math.cos(math.pi * alpha))
            
            position = target * smooth_alpha
            self.send_command(joint, position)
            self.rate.sleep()
        
        # Hold position
        for _ in range(25):  # 0.5 seconds
            self.send_command(joint, target)
            self.rate.sleep()
    
    def test_joint_limits(self, joint):
        """Test a joint through its safe range"""
        if joint not in self.limits:
            rospy.logerr(f"Unknown joint: {joint}")
            return
        
        min_pos, max_pos = self.limits[joint]
        
        rospy.loginfo(f"Testing {joint} limits: {min_pos:.2f} to {max_pos:.2f} rad")
        
        # Return to neutral first
        self.move_to_position(joint, 0.0, 1.5)
        
        # Test positive limit
        self.move_to_position(joint, max_pos, 2.0)
        time.sleep(1.0)
        
        # Test negative limit
        self.move_to_position(joint, min_pos, 3.0)
        time.sleep(1.0)
        
        # Return to neutral
        self.move_to_position(joint, 0.0, 2.0)
        
        rospy.loginfo(f"Completed {joint} limit test")
    
    def test_all_joints(self):
        """Test all joints sequentially"""
        joints = ["Joint01", "Joint02", "Joint03", "Joint04", "Joint05", "Joint06", "Gripper"]
        
        rospy.loginfo("Starting comprehensive joint limit test")
        
        for joint in joints:
            if rospy.is_shutdown():
                break
            
            rospy.loginfo(f"\\n--- Testing {joint} ---")
            self.test_joint_limits(joint)
            time.sleep(1.0)
        
        rospy.loginfo("All joint tests completed")
    
    def sine_sweep_all(self, duration=30.0, frequency=0.2):
        """Perform sine sweep on all joints simultaneously"""
        rospy.loginfo(f"Starting {duration}s sine sweep at {frequency}Hz")
        
        start_time = time.time()
        
        while not rospy.is_shutdown():
            elapsed = time.time() - start_time
            if elapsed >= duration:
                break
            
            # Calculate sine wave
            phase = 2 * math.pi * frequency * elapsed
            
            for joint, (min_pos, max_pos) in self.limits.items():
                # Scale sine wave to joint limits
                amplitude = (max_pos - min_pos) / 2.0 * 0.8  # 80% of range
                center = (max_pos + min_pos) / 2.0
                position = center + amplitude * math.sin(phase)
                
                self.send_command(joint, position)
            
            self.rate.sleep()
        
        # Return all to neutral
        rospy.loginfo("Returning to neutral positions")
        for _ in range(100):  # 2 seconds
            for joint in self.limits.keys():
                self.send_command(joint, 0.0)
            self.rate.sleep()

def main():
    parser = argparse.ArgumentParser(description="Z1 Safe Limits Tester")
    parser.add_argument("--mode", type=int, default=10, help="Controller mode")
    parser.add_argument("--kp", type=float, default=35.0, help="Proportional gain")
    parser.add_argument("--kd", type=float, default=1.5, help="Derivative gain")
    parser.add_argument("--test", choices=["individual", "sweep", "both"], 
                       default="both", help="Test type")
    parser.add_argument("--joint", help="Test specific joint only")
    parser.add_argument("--duration", type=float, default=30.0, 
                       help="Sweep duration (seconds)")
    args = parser.parse_args()
    
    try:
        tester = Z1SafeLimits(args.mode, args.kp, args.kd)
        
        if args.joint:
            # Test specific joint
            tester.test_joint_limits(args.joint)
        elif args.test == "individual":
            tester.test_all_joints()
        elif args.test == "sweep":
            tester.sine_sweep_all(args.duration)
        else:  # both
            tester.test_all_joints()
            time.sleep(2.0)
            tester.sine_sweep_all(args.duration)
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Safe limits test interrupted")

if __name__ == "__main__":
    main()