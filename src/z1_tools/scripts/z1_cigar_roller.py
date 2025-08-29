#!/usr/bin/env python3
"""
Z1 Cigar Roller
Simulates the art of hand-rolling cigars with precise movements
"""
import rospy, math, time, argparse
from unitree_legged_msgs.msg import MotorCmd

class Z1CigarRoller:
    def __init__(self, mode=10, kp=35.0, kd=1.5):
        rospy.init_node("z1_cigar_roller")
        
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
        
        # Cigar rolling poses
        self.poses = {
            "inspect_leaf": [0.3, -0.2, 0.6, 0.0, -0.4, 0.0, 0.4],
            "prepare_table": [0.0, -0.4, 1.0, 0.0, -0.6, 0.0, 0.8],
            "rolling_start": [-0.2, -0.3, 0.9, 0.0, -0.6, 0.0, 0.3],
            "rolling_end": [0.2, -0.3, 0.9, 0.0, -0.6, 0.0, 0.3],
            "quality_check": [0.0, -0.1, 0.4, 0.0, -0.3, 0.0, 0.5],
            "final_trim": [0.1, -0.2, 0.7, 0.0, -0.5, 0.0, 0.2]
        }
    
    def send_pose(self, pose_name_or_positions, duration=2.0):
        """Move to pose smoothly"""
        if isinstance(pose_name_or_positions, str):
            if pose_name_or_positions in self.poses:
                target = self.poses[pose_name_or_positions]
            else:
                rospy.logerr(f"Unknown pose: {pose_name_or_positions}")
                return
        else:
            target = pose_name_or_positions
        
        start_time = time.time()
        while not rospy.is_shutdown():
            elapsed = time.time() - start_time
            if elapsed >= duration:
                break
            
            alpha = elapsed / duration
            smooth_alpha = 0.5 * (1 - math.cos(math.pi * alpha))
            
            joints = ["Joint01", "Joint02", "Joint03", "Joint04", "Joint05", "Joint06", "Gripper"]
            for i, joint in enumerate(joints):
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
    
    def inspect_tobacco_leaf(self):
        """Carefully inspect tobacco leaf quality"""
        rospy.loginfo("🍃 Inspecting tobacco leaf quality...")
        
        # Move to inspection position
        self.send_pose("inspect_leaf", 2.0)
        
        # Detailed inspection movements
        base_pose = self.poses["inspect_leaf"]
        
        # Rotate to examine different angles
        for angle in [-0.3, 0.0, 0.3, 0.0]:
            inspect_pose = base_pose.copy()
            inspect_pose[0] = angle  # Base rotation
            inspect_pose[5] = angle * 0.5  # Wrist follows
            self.send_pose(inspect_pose, 1.0)
        
        # Close examination - bring closer
        close_inspect = base_pose.copy()
        close_inspect[1] += 0.1  # Shoulder forward
        close_inspect[2] -= 0.2  # Elbow bend more
        self.send_pose(close_inspect, 1.5)
        
        rospy.loginfo("✅ Leaf quality approved!")
    
    def prepare_rolling_surface(self):
        """Prepare the rolling surface"""
        rospy.loginfo("🛠️ Preparing rolling surface...")
        
        # Move to table preparation
        self.send_pose("prepare_table", 2.0)
        
        # Smooth the surface with gentle sweeping motions
        base_pose = self.poses["prepare_table"]
        
        for sweep in range(3):
            # Left to right sweep
            for x in [-0.3, 0.0, 0.3]:
                sweep_pose = base_pose.copy()
                sweep_pose[0] = x
                sweep_pose[1] -= 0.1  # Lower to surface
                self.send_pose(sweep_pose, 0.8)
        
        rospy.loginfo("✨ Surface prepared perfectly!")
    
    def roll_cigar(self, rolls=8):
        """Execute the precise rolling motion"""
        rospy.loginfo(f"🚬 Rolling cigar with {rolls} precise rolls...")
        
        # Start rolling position
        self.send_pose("rolling_start", 2.0)
        
        for roll_num in range(rolls):
            rospy.loginfo(f"Roll {roll_num + 1}/{rolls}")
            
            # Rolling motion - smooth back and forth with rotation
            start_time = time.time()
            roll_duration = 3.0
            
            while time.time() - start_time < roll_duration:
                t = time.time() - start_time
                progress = t / roll_duration
                
                # Sinusoidal rolling motion
                base_pos = -0.2 + 0.4 * progress  # Left to right
                roll_phase = 2 * math.pi * 2 * progress  # 2 full rotations per roll
                
                # Add rolling texture with wrist and forearm
                wrist_roll = 0.3 * math.sin(roll_phase)
                forearm_roll = 0.2 * math.sin(roll_phase * 1.5)
                
                # Slight pressure variation
                pressure_depth = 0.05 * math.sin(roll_phase * 0.5)
                
                roll_pose = [
                    base_pos,                           # Base slides left to right
                    -0.3 - pressure_depth,             # Shoulder with pressure
                    0.9 + pressure_depth * 2,          # Elbow compensates
                    forearm_roll,                       # Forearm rolling
                    -0.6,                              # Wrist pitch steady
                    wrist_roll,                        # Wrist roll for texture
                    0.3                                # Gentle grip
                ]
                
                self.send_pose(roll_pose, 0.1)
            
            # Brief pause between rolls
            time.sleep(0.5)
        
        rospy.loginfo("🎯 Rolling complete - perfect cylinder achieved!")
    
    def quality_inspection(self):
        """Inspect the rolled cigar quality"""
        rospy.loginfo("🔍 Conducting quality inspection...")
        
        # Move to inspection position
        self.send_pose("quality_check", 2.0)
        
        # Rotate cigar for 360° inspection
        base_pose = self.poses["quality_check"]
        
        for angle in [0, 90, 180, 270, 360]:
            inspect_pose = base_pose.copy()
            inspect_pose[5] = math.radians(angle) * 0.5  # Wrist rotation
            inspect_pose[0] = math.sin(math.radians(angle)) * 0.1  # Slight base movement
            self.send_pose(inspect_pose, 1.0)
        
        # Check firmness with gentle squeeze
        firmness_pose = base_pose.copy()
        firmness_pose[6] = 0.1  # Gentle squeeze
        self.send_pose(firmness_pose, 1.0)
        
        # Release
        firmness_pose[6] = 0.5
        self.send_pose(firmness_pose, 0.5)
        
        rospy.loginfo("✅ Quality inspection passed - excellent construction!")
    
    def final_trimming(self):
        """Perform final trimming of cigar ends"""
        rospy.loginfo("✂️ Performing final precision trimming...")
        
        # Move to trimming position
        self.send_pose("final_trim", 2.0)
        
        # Precise trimming motions
        base_pose = self.poses["final_trim"]
        
        # Trim first end
        for cut in range(2):
            trim_pose = base_pose.copy()
            trim_pose[6] = 0.1 if cut == 0 else 0.8  # Close then open (cutting motion)
            self.send_pose(trim_pose, 0.3)
        
        # Rotate cigar
        rotate_pose = base_pose.copy()
        rotate_pose[5] = math.pi  # 180° wrist rotation
        self.send_pose(rotate_pose, 1.5)
        
        # Trim second end
        for cut in range(2):
            trim_pose = rotate_pose.copy()
            trim_pose[6] = 0.1 if cut == 0 else 0.8
            self.send_pose(trim_pose, 0.3)
        
        rospy.loginfo("🎉 Masterpiece complete - artisan cigar ready!")
    
    def master_craftsman_sequence(self):
        """Complete cigar rolling sequence like a master craftsman"""
        rospy.loginfo("🎭 Beginning master craftsman cigar rolling sequence...")
        
        # Complete artisan process
        self.inspect_tobacco_leaf()
        time.sleep(1.0)
        
        self.prepare_rolling_surface()
        time.sleep(1.0)
        
        self.roll_cigar(rolls=6)
        time.sleep(1.0)
        
        self.quality_inspection()
        time.sleep(1.0)
        
        self.final_trimming()
        time.sleep(1.0)
        
        # Final presentation
        rospy.loginfo("🏆 Presenting the finished masterpiece!")
        presentation_pose = [0.0, 0.2, 0.3, 0.0, 0.1, 0.0, 0.5]
        self.send_pose(presentation_pose, 3.0)
        
        # Proud craftsman gesture
        for _ in range(2):
            self.send_pose([0.0, 0.4, 0.1, 0.0, 0.3, 0.0, 0.5], 1.0)
            self.send_pose(presentation_pose, 1.0)
    
    def return_home(self):
        """Return to neutral position"""
        rospy.loginfo("Returning to workshop rest position")
        self.send_pose([0.0] * 7, 3.0)

def main():
    parser = argparse.ArgumentParser(description="Z1 Master Cigar Roller")
    parser.add_argument("--mode", type=int, default=10)
    parser.add_argument("--kp", type=float, default=35.0)
    parser.add_argument("--kd", type=float, default=1.5)
    parser.add_argument("--step", choices=["inspect", "prepare", "roll", "quality", "trim", "full"],
                       default="full", help="Specific step or full sequence")
    parser.add_argument("--rolls", type=int, default=6, help="Number of rolling passes")
    args = parser.parse_args()
    
    try:
        roller = Z1CigarRoller(args.mode, args.kp, args.kd)
        
        if args.step == "inspect":
            roller.inspect_tobacco_leaf()
        elif args.step == "prepare":
            roller.prepare_rolling_surface()
        elif args.step == "roll":
            roller.roll_cigar(args.rolls)
        elif args.step == "quality":
            roller.quality_inspection()
        elif args.step == "trim":
            roller.final_trimming()
        else:  # full
            roller.master_craftsman_sequence()
        
        roller.return_home()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Cigar rolling interrupted")

if __name__ == "__main__":
    main()