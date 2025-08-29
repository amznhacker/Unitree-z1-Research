#!/usr/bin/env python3
"""
Z1 Pick and Place Demo
Safe pick and place sequence with predefined waypoints
"""
import rospy, math, time, argparse
from unitree_legged_msgs.msg import MotorCmd

class Z1PickPlace:
    def __init__(self, mode=10, kp=35.0, kd=1.5):
        rospy.init_node("z1_pick_place")
        
        self.mode = mode
        self.kp = kp
        self.kd = kd
        
        # Publishers
        self.pubs = {}
        joints = ["Joint01", "Joint02", "Joint03", "Joint04", "Joint05", "Joint06", "Gripper"]
        for joint in joints:
            topic = f"/z1_gazebo/{joint}_controller/command"
            self.pubs[joint] = rospy.Publisher(topic, MotorCmd, queue_size=10)
        
        self.rate = rospy.Rate(50)
        
        # Predefined safe poses
        self.poses = {
            "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "ready": [0.0, -0.5, 0.8, 0.0, -0.3, 0.0, 0.0],
            "pick_approach": [0.5, -0.3, 1.2, 0.0, -0.9, 0.0, 0.8],
            "pick_grasp": [0.5, -0.1, 1.4, 0.0, -1.3, 0.0, 0.8],
            "pick_lift": [0.5, -0.3, 1.2, 0.0, -0.9, 0.0, 0.2],
            "place_approach": [-0.5, -0.3, 1.2, 0.0, -0.9, 0.0, 0.2],
            "place_drop": [-0.5, -0.1, 1.4, 0.0, -1.3, 0.0, 0.2],
            "place_release": [-0.5, -0.1, 1.4, 0.0, -1.3, 0.0, 0.8]
        }
    
    def send_pose(self, pose_name, duration=2.0):
        """Move to predefined pose over specified duration"""
        if pose_name not in self.poses:
            rospy.logerr(f"Unknown pose: {pose_name}")
            return False
        
        target = self.poses[pose_name]
        joints = ["Joint01", "Joint02", "Joint03", "Joint04", "Joint05", "Joint06", "Gripper"]
        
        rospy.loginfo(f"Moving to pose: {pose_name}")
        
        # Get current positions (assume starting from previous target)
        start_time = time.time()
        
        while not rospy.is_shutdown():
            elapsed = time.time() - start_time
            if elapsed >= duration:
                break
            
            # Linear interpolation
            alpha = elapsed / duration
            alpha = min(1.0, alpha)
            
            # Smooth interpolation using cosine
            smooth_alpha = 0.5 * (1 - math.cos(math.pi * alpha))
            
            for i, joint in enumerate(joints):
                # Simple interpolation from 0 to target (assumes starting from neutral)
                position = target[i] * smooth_alpha
                
                msg = MotorCmd()
                msg.mode = self.mode
                msg.q = float(position)
                msg.dq = 0.0
                msg.tau = 0.0
                msg.Kp = self.kp
                msg.Kd = self.kd
                
                self.pubs[joint].publish(msg)
            
            self.rate.sleep()
        
        # Hold final position briefly
        for _ in range(25):  # 0.5 seconds at 50Hz
            for i, joint in enumerate(joints):
                msg = MotorCmd()
                msg.mode = self.mode
                msg.q = float(target[i])
                msg.dq = 0.0
                msg.tau = 0.0
                msg.Kp = self.kp
                msg.Kd = self.kd
                self.pubs[joint].publish(msg)
            self.rate.sleep()
        
        return True
    
    def execute_pick_place(self):
        """Execute complete pick and place sequence"""
        sequence = [
            ("home", 2.0),
            ("ready", 2.0),
            ("pick_approach", 3.0),
            ("pick_grasp", 2.0),
            ("pick_lift", 2.0),
            ("place_approach", 3.0),
            ("place_drop", 2.0),
            ("place_release", 1.0),
            ("ready", 2.0),
            ("home", 2.0)
        ]
        
        rospy.loginfo("Starting pick and place sequence...")
        
        for pose_name, duration in sequence:
            if rospy.is_shutdown():
                break
            
            success = self.send_pose(pose_name, duration)
            if not success:
                rospy.logerr("Failed to reach pose, aborting sequence")
                break
            
            rospy.loginfo(f"Completed pose: {pose_name}")
            time.sleep(0.5)  # Brief pause between poses
        
        rospy.loginfo("Pick and place sequence completed")

def main():
    parser = argparse.ArgumentParser(description="Z1 Pick and Place Demo")
    parser.add_argument("--mode", type=int, default=10, help="Controller mode")
    parser.add_argument("--kp", type=float, default=35.0, help="Proportional gain")
    parser.add_argument("--kd", type=float, default=1.5, help="Derivative gain")
    parser.add_argument("--cycles", type=int, default=1, help="Number of pick-place cycles")
    args = parser.parse_args()
    
    try:
        controller = Z1PickPlace(args.mode, args.kp, args.kd)
        
        for cycle in range(args.cycles):
            rospy.loginfo(f"Starting cycle {cycle + 1}/{args.cycles}")
            controller.execute_pick_place()
            
            if cycle < args.cycles - 1:
                rospy.loginfo("Waiting 3 seconds before next cycle...")
                time.sleep(3.0)
        
        rospy.loginfo("All cycles completed")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Pick and place interrupted")

if __name__ == "__main__":
    main()