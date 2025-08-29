#!/usr/bin/env python3
"""
Z1 Chess Player
Simulates playing chess with piece movements and gestures
"""
import rospy, math, time, random, argparse
from unitree_legged_msgs.msg import MotorCmd

class Z1ChessPlayer:
    def __init__(self, mode=10, kp=35.0, kd=1.5):
        rospy.init_node("z1_chess_player")
        
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
        
        # Chess board positions (8x8 grid)
        self.board_positions = self.generate_board_positions()
        
        # Chess poses
        self.poses = {
            "thinking": [0.0, -0.8, 1.4, 0.0, -0.6, 0.0, 0.0],  # Hand to chin
            "ready": [0.0, -0.3, 0.8, 0.0, -0.5, 0.0, 0.8],     # Above board
            "victory": [0.0, 0.5, 0.2, 0.0, 0.3, 0.0, 0.0],     # Arms up celebration
            "defeat": [0.0, -1.0, 1.8, 0.0, -0.8, 0.0, 0.0]     # Head in hands
        }
    
    def generate_board_positions(self):
        """Generate 8x8 chess board positions in joint space"""
        positions = {}
        for row in range(8):
            for col in range(8):
                # Map board to arm workspace
                base_angle = (col - 3.5) * 0.15  # -0.525 to +0.525 rad
                reach_distance = 0.3 + row * 0.05  # Varying reach
                
                # Calculate joint angles for position
                shoulder = -0.2 - reach_distance * 0.8
                elbow = 0.8 + reach_distance * 1.2
                wrist = -shoulder - elbow * 0.5
                
                square = f"{chr(ord('a') + col)}{row + 1}"
                positions[square] = [base_angle, shoulder, elbow, 0.0, wrist, 0.0]
        
        return positions
    
    def send_pose(self, pose_name_or_positions, duration=2.0):
        """Move to pose or position"""
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
    
    def thinking_gesture(self, duration=5.0):
        """Contemplative thinking pose"""
        rospy.loginfo("🤔 Thinking about next move...")
        
        # Move to thinking pose
        self.send_pose("thinking", 2.0)
        
        # Subtle head movements while thinking
        start_time = time.time()
        base_pose = self.poses["thinking"]
        
        while time.time() - start_time < duration:
            t = time.time() - start_time
            
            # Subtle wrist and base movements
            wrist_think = 0.1 * math.sin(t * 2)
            base_think = 0.05 * math.sin(t * 1.5)
            
            thinking_pose = base_pose.copy()
            thinking_pose[0] += base_think  # Base
            thinking_pose[5] += wrist_think  # Wrist roll
            
            self.send_pose(thinking_pose, 0.1)
    
    def make_chess_move(self, from_square, to_square, piece="pawn"):
        """Execute a chess move from one square to another"""
        rospy.loginfo(f"♟️ Moving {piece} from {from_square} to {to_square}")
        
        if from_square not in self.board_positions or to_square not in self.board_positions:
            rospy.logerr("Invalid chess square!")
            return
        
        # Move to ready position
        self.send_pose("ready", 1.5)
        
        # Approach source square
        from_pos = self.board_positions[from_square].copy()
        from_pos[6] = 0.8  # Open gripper
        self.send_pose(from_pos, 2.0)
        
        # Lower to pick up piece
        pickup_pos = from_pos.copy()
        pickup_pos[1] -= 0.1  # Lower shoulder
        pickup_pos[2] += 0.2  # Adjust elbow
        self.send_pose(pickup_pos, 1.0)
        
        # Close gripper to grab piece
        pickup_pos[6] = 0.2  # Close gripper
        self.send_pose(pickup_pos, 0.5)
        
        # Lift piece
        lift_pos = from_pos.copy()
        lift_pos[6] = 0.2  # Keep gripper closed
        self.send_pose(lift_pos, 1.0)
        
        # Move to destination square
        to_pos = self.board_positions[to_square].copy()
        to_pos[6] = 0.2  # Keep gripper closed
        self.send_pose(to_pos, 2.5)
        
        # Lower to place piece
        place_pos = to_pos.copy()
        place_pos[1] -= 0.1  # Lower shoulder
        place_pos[2] += 0.2  # Adjust elbow
        self.send_pose(place_pos, 1.0)
        
        # Release piece
        place_pos[6] = 0.8  # Open gripper
        self.send_pose(place_pos, 0.5)
        
        # Return to ready position
        self.send_pose("ready", 1.5)
    
    def victory_dance(self):
        """Celebrate winning the game"""
        rospy.loginfo("🎉 Victory! Checkmate!")
        
        # Victory pose
        self.send_pose("victory", 2.0)
        
        # Victory dance
        for _ in range(3):
            # Raise arms higher
            victory_high = [0.0, 0.8, -0.2, 0.0, 0.6, 0.0, 0.0]
            self.send_pose(victory_high, 0.8)
            
            # Back to victory pose
            self.send_pose("victory", 0.8)
        
        # Final flourish with wrist rolls
        for _ in range(4):
            flourish = self.poses["victory"].copy()
            flourish[5] = 1.2 if _ % 2 == 0 else -1.2
            self.send_pose(flourish, 0.5)
    
    def defeat_gesture(self):
        """Show disappointment at losing"""
        rospy.loginfo("😞 Defeated... Good game!")
        
        # Slow move to defeat pose
        self.send_pose("defeat", 3.0)
        
        # Shake head in disappointment
        base_pose = self.poses["defeat"]
        for _ in range(3):
            shake_left = base_pose.copy()
            shake_left[0] = -0.2
            self.send_pose(shake_left, 0.6)
            
            shake_right = base_pose.copy()
            shake_right[0] = 0.2
            self.send_pose(shake_right, 0.6)
        
        # Return to center
        self.send_pose("defeat", 1.0)
    
    def play_sample_game(self):
        """Play through a sample chess game"""
        moves = [
            ("e2", "e4", "pawn"),
            ("e7", "e5", "pawn"),
            ("g1", "f3", "knight"),
            ("b8", "c6", "knight"),
            ("f1", "c4", "bishop"),
            ("f8", "c5", "bishop"),
            ("d2", "d3", "pawn"),
            ("d7", "d6", "pawn")
        ]
        
        rospy.loginfo("🏁 Starting chess game simulation...")
        
        # Initial thinking
        self.thinking_gesture(3.0)
        
        for i, (from_sq, to_sq, piece) in enumerate(moves):
            if rospy.is_shutdown():
                break
            
            rospy.loginfo(f"Move {i+1}: {piece} {from_sq} to {to_sq}")
            
            # Think before each move
            if i > 0:
                self.thinking_gesture(2.0)
            
            # Make the move
            self.make_chess_move(from_sq, to_sq, piece)
            
            # Brief pause
            time.sleep(1.0)
        
        # Random outcome
        if random.choice([True, False]):
            self.victory_dance()
        else:
            self.defeat_gesture()
    
    def return_home(self):
        """Return to neutral position"""
        rospy.loginfo("Returning to home position")
        self.send_pose([0.0] * 7, 3.0)

def main():
    parser = argparse.ArgumentParser(description="Z1 Chess Player")
    parser.add_argument("--mode", type=int, default=10)
    parser.add_argument("--kp", type=float, default=35.0)
    parser.add_argument("--kd", type=float, default=1.5)
    parser.add_argument("--move", nargs=2, help="Make single move: from_square to_square")
    parser.add_argument("--gesture", choices=["thinking", "victory", "defeat"], 
                       help="Perform specific gesture")
    args = parser.parse_args()
    
    try:
        player = Z1ChessPlayer(args.mode, args.kp, args.kd)
        
        if args.move:
            from_sq, to_sq = args.move
            player.make_chess_move(from_sq, to_sq)
        elif args.gesture:
            if args.gesture == "thinking":
                player.thinking_gesture()
            elif args.gesture == "victory":
                player.victory_dance()
            elif args.gesture == "defeat":
                player.defeat_gesture()
        else:
            player.play_sample_game()
        
        player.return_home()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Chess game interrupted")

if __name__ == "__main__":
    main()