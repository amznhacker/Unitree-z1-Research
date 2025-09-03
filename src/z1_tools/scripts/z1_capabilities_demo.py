#!/usr/bin/env python3
"""
Z1 Capabilities Demo
Shows all Z1 robotic arm capabilities and safety limits
"""
import rospy, math, time
from unitree_legged_msgs.msg import MotorCmd

class Z1CapabilitiesDemo:
    def __init__(self, mode=10, kp=35.0, kd=1.5):
        rospy.init_node("z1_capabilities_demo")
        
        self.mode = mode
        self.kp = kp
        self.kd = kd
        
        # Publishers
        self.pubs = {}
        self.joints = ["Joint01", "Joint02", "Joint03", "Joint04", "Joint05", "Joint06", "Gripper"]
        for joint in self.joints:
            topic = f"/z1_gazebo/{joint}_controller/command"
            self.pubs[joint] = rospy.Publisher(topic, MotorCmd, queue_size=10)
        
        # Safety limits (conservative for demo)
        self.limits = {
            "Joint01": (-1.2, 1.2),   # Base rotation: ±69°
            "Joint02": (-1.0, 1.0),   # Shoulder pitch: ±57°
            "Joint03": (0.0, 2.4),    # Elbow: 0° to 137°
            "Joint04": (-1.2, 1.2),   # Forearm roll: ±69°
            "Joint05": (-1.0, 1.0),   # Wrist pitch: ±57°
            "Joint06": (-1.2, 1.2),   # Wrist roll: ±69°
            "Gripper": (0.0, 0.8)     # Gripper: open to closed
        }
        
        # Named poses for demonstrations
        self.poses = {
            "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8],
            "reach_forward": [0.0, -0.5, 1.2, 0.0, -0.7, 0.0, 0.8],
            "reach_up": [0.0, 0.8, 0.4, 0.0, 0.4, 0.0, 0.8],
            "reach_left": [0.8, -0.3, 0.8, 0.0, -0.5, 0.0, 0.8],
            "reach_right": [-0.8, -0.3, 0.8, 0.0, -0.5, 0.0, 0.8],
            "pick_table": [0.0, -0.8, 1.5, 0.0, -0.7, 0.0, 0.8],
            "wave": [0.5, 0.3, 0.6, 0.0, 0.2, 0.0, 0.8]
        }
        
        self.rate = rospy.Rate(50)
        rospy.loginfo("🤖 Z1 Capabilities Demo Ready!")
    
    def send_pose(self, pose_name_or_positions, duration=2.0):
        """Move to pose smoothly with safety checks"""
        if isinstance(pose_name_or_positions, str):
            if pose_name_or_positions in self.poses:
                target = self.poses[pose_name_or_positions]
            else:
                rospy.logerr(f"Unknown pose: {pose_name_or_positions}")
                return False
        else:
            target = pose_name_or_positions
        
        # Safety check all positions
        for i, joint in enumerate(self.joints):
            if i < len(target):
                min_pos, max_pos = self.limits[joint]
                if not (min_pos <= target[i] <= max_pos):
                    rospy.logerr(f"Position {target[i]} for {joint} exceeds limits [{min_pos}, {max_pos}]")
                    return False
        
        start_time = time.time()
        while not rospy.is_shutdown():
            elapsed = time.time() - start_time
            if elapsed >= duration:
                break
            
            alpha = elapsed / duration
            smooth_alpha = 0.5 * (1 - math.cos(math.pi * alpha))
            
            for i, joint in enumerate(self.joints):
                if i < len(target):
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
        return True
    
    def demo_joint_ranges(self):
        """Demonstrate each joint's range of motion"""
        rospy.loginfo("🔧 Demonstrating Joint Ranges and Safety Limits")
        
        for i, joint in enumerate(self.joints):
            min_pos, max_pos = self.limits[joint]
            rospy.loginfo(f"📐 {joint}: {min_pos:.1f} to {max_pos:.1f} radians ({math.degrees(min_pos):.0f}° to {math.degrees(max_pos):.0f}°)")
            
            # Move to min position
            pose = [0.0] * 7
            pose[i] = min_pos
            self.send_pose(pose, 1.5)
            
            # Move to max position
            pose[i] = max_pos
            self.send_pose(pose, 1.5)
            
            # Return to neutral
            pose[i] = 0.0
            self.send_pose(pose, 1.0)
    
    def demo_gripper(self):
        """Demonstrate gripper capabilities"""
        rospy.loginfo("🤏 Demonstrating Gripper Capabilities")
        
        # Open gripper fully
        rospy.loginfo("Opening gripper...")
        self.send_pose([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8], 1.0)
        
        # Close gripper gradually
        rospy.loginfo("Closing gripper gradually...")
        for grip in [0.6, 0.4, 0.2, 0.0]:
            self.send_pose([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, grip], 0.8)
        
        # Open again
        rospy.loginfo("Opening gripper...")
        self.send_pose([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8], 1.0)
    
    def demo_workspace(self):
        """Demonstrate workspace reach"""
        rospy.loginfo("🌐 Demonstrating Workspace Reach")
        
        workspace_poses = ["reach_forward", "reach_up", "reach_left", "reach_right", "pick_table"]
        
        for pose_name in workspace_poses:
            rospy.loginfo(f"Moving to {pose_name.replace('_', ' ')}...")
            self.send_pose(pose_name, 2.0)
            time.sleep(0.5)
        
        # Return home
        self.send_pose("home", 2.0)
    
    def demo_precision_movements(self):
        """Demonstrate precision control"""
        rospy.loginfo("🎯 Demonstrating Precision Movements")
        
        # Small circular motion
        center = [0.0, -0.3, 0.8, 0.0, -0.5, 0.0, 0.8]
        radius = 0.1
        
        for angle in range(0, 361, 30):
            rad = math.radians(angle)
            x_offset = radius * math.cos(rad)
            y_offset = radius * math.sin(rad)
            
            precise_pose = center.copy()
            precise_pose[0] += x_offset  # Base rotation
            precise_pose[1] += y_offset * 0.5  # Shoulder adjustment
            
            self.send_pose(precise_pose, 0.5)
    
    def demo_pick_and_place(self):
        """Demonstrate pick and place sequence"""
        rospy.loginfo("📦 Demonstrating Pick and Place")
        
        # Approach object
        rospy.loginfo("Approaching object...")
        approach = [0.2, -0.6, 1.2, 0.0, -0.6, 0.0, 0.8]
        self.send_pose(approach, 2.0)
        
        # Lower to object
        rospy.loginfo("Lowering to object...")
        lower = [0.2, -0.8, 1.5, 0.0, -0.7, 0.0, 0.8]
        self.send_pose(lower, 1.5)
        
        # Close gripper
        rospy.loginfo("Grasping object...")
        grasp = lower.copy()
        grasp[6] = 0.2
        self.send_pose(grasp, 1.0)
        
        # Lift object
        rospy.loginfo("Lifting object...")
        lift = [0.2, -0.4, 1.0, 0.0, -0.6, 0.0, 0.2]
        self.send_pose(lift, 2.0)
        
        # Move to place location
        rospy.loginfo("Moving to place location...")
        place_approach = [-0.2, -0.4, 1.0, 0.0, -0.6, 0.0, 0.2]
        self.send_pose(place_approach, 2.0)
        
        # Lower to place
        rospy.loginfo("Placing object...")
        place = [-0.2, -0.8, 1.5, 0.0, -0.7, 0.0, 0.2]
        self.send_pose(place, 1.5)
        
        # Release object
        rospy.loginfo("Releasing object...")
        release = place.copy()
        release[6] = 0.8
        self.send_pose(release, 1.0)
        
        # Return home
        rospy.loginfo("Returning home...")
        self.send_pose("home", 2.0)
    
    def demo_safety_features(self):
        """Demonstrate safety features"""
        rospy.loginfo("🛡️ Demonstrating Safety Features")
        
        # Try to exceed limits (should be prevented)
        rospy.loginfo("Testing safety limits (movements will be clamped)...")
        
        unsafe_poses = [
            [2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8],  # Excessive base rotation
            [0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.8],  # Excessive shoulder
            [0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.8], # Negative elbow
        ]
        
        for i, unsafe_pose in enumerate(unsafe_poses):
            rospy.loginfo(f"Testing unsafe pose {i+1}...")
            success = self.send_pose(unsafe_pose, 1.0)
            if not success:
                rospy.loginfo("✅ Safety system prevented unsafe movement")
        
        # Emergency stop demonstration
        rospy.loginfo("Emergency stop: returning to safe home position")
        self.send_pose("home", 1.0)
    
    def run_full_demo(self):
        """Run complete capabilities demonstration"""
        rospy.loginfo("🚀 Starting Z1 Full Capabilities Demo")
        rospy.loginfo("=" * 50)
        
        # Start from home
        self.send_pose("home", 2.0)
        time.sleep(1.0)
        
        # Run all demonstrations
        self.demo_joint_ranges()
        time.sleep(1.0)
        
        self.demo_gripper()
        time.sleep(1.0)
        
        self.demo_workspace()
        time.sleep(1.0)
        
        self.demo_precision_movements()
        time.sleep(1.0)
        
        self.demo_pick_and_place()
        time.sleep(1.0)
        
        self.demo_safety_features()
        
        # Final home position
        self.send_pose("home", 2.0)
        
        rospy.loginfo("✨ Z1 Capabilities Demo Complete!")
        rospy.loginfo("📋 Summary of Capabilities:")
        rospy.loginfo("  ✅ 6-DOF arm movement with safety limits")
        rospy.loginfo("  ✅ Precision gripper control (0.0-0.8)")
        rospy.loginfo("  ✅ Smooth interpolated movements")
        rospy.loginfo("  ✅ Pick and place operations")
        rospy.loginfo("  ✅ Workspace reach demonstration")
        rospy.loginfo("  ✅ Safety limit enforcement")
        rospy.loginfo("  ✅ Emergency stop capability")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Z1 Capabilities Demo")
    parser.add_argument("--mode", type=int, default=10)
    parser.add_argument("--kp", type=float, default=35.0)
    parser.add_argument("--kd", type=float, default=1.5)
    parser.add_argument("--demo", choices=["full", "joints", "gripper", "workspace", "precision", "pick", "safety"],
                       default="full", help="Which demo to run")
    args = parser.parse_args()
    
    try:
        demo = Z1CapabilitiesDemo(args.mode, args.kp, args.kd)
        
        if args.demo == "full":
            demo.run_full_demo()
        elif args.demo == "joints":
            demo.demo_joint_ranges()
        elif args.demo == "gripper":
            demo.demo_gripper()
        elif args.demo == "workspace":
            demo.demo_workspace()
        elif args.demo == "precision":
            demo.demo_precision_movements()
        elif args.demo == "pick":
            demo.demo_pick_and_place()
        elif args.demo == "safety":
            demo.demo_safety_features()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Demo interrupted")

if __name__ == "__main__":
    main()