#!/usr/bin/env python3
"""
Z1 Magician
Perform magic tricks and illusions
"""
import rospy, math, time, random, argparse
from unitree_legged_msgs.msg import MotorCmd

class Z1Magician:
    def __init__(self, mode=10, kp=35.0, kd=1.5):
        rospy.init_node("z1_magician")
        
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
        
        # Magic poses
        self.poses = {
            "abracadabra": [0.0, 0.5, 0.2, 0.0, 0.3, 0.0, 0.8],
            "hat_tip": [0.3, -0.1, 0.4, 0.0, -0.3, 0.0, 0.5],
            "wand_flourish": [0.0, 0.2, 0.5, 0.0, 0.1, 0.0, 0.2],
            "reveal_pose": [0.0, 0.0, 0.6, 0.0, -0.6, 0.0, 0.8],
            "mysterious": [-0.2, -0.3, 0.8, 0.0, -0.5, 0.0, 0.3],
            "ta_da": [0.0, 0.6, 0.1, 0.0, 0.5, 0.0, 0.8]
        }
    
    def send_pose(self, pose_name_or_positions, duration=2.0):
        """Move to pose with magical smoothness"""
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
    
    def magic_introduction(self):
        """Dramatic introduction"""
        rospy.loginfo("🎩 Ladies and gentlemen, prepare to be amazed!")
        
        # Tip hat
        self.send_pose("hat_tip", 2.0)
        time.sleep(1.0)
        
        # Mysterious gesture
        self.send_pose("mysterious", 2.0)
        time.sleep(1.0)
        
        rospy.loginfo("✨ The magic begins...")
    
    def disappearing_act(self):
        """Make something 'disappear'"""
        rospy.loginfo("🪄 For my first trick... the disappearing object!")
        
        # Show object in hand
        rospy.loginfo("👀 Observe this object carefully...")
        show_object = [0.0, -0.1, 0.5, 0.0, -0.4, 0.0, 0.2]  # Closed grip
        self.send_pose(show_object, 2.0)
        
        # Wave other hand mysteriously
        wave_pose = [0.3, 0.1, 0.4, 0.0, 0.2, 0.0, 0.8]
        self.send_pose(wave_pose, 1.5)
        
        # Magic words with wand flourish
        rospy.loginfo("🎭 Abracadabra! Alakazam!")
        for _ in range(3):
            flourish_up = [0.0, 0.3, 0.3, 0.0, 0.4, 0.5, 0.2]
            flourish_down = [0.0, -0.1, 0.7, 0.0, -0.3, -0.5, 0.2]
            self.send_pose(flourish_up, 0.5)
            self.send_pose(flourish_down, 0.5)
        
        # Dramatic pause
        self.send_pose("mysterious", 1.0)
        time.sleep(2.0)
        
        # Reveal empty hand
        rospy.loginfo("🤯 POOF! It's gone!")
        empty_hand = [0.0, -0.1, 0.5, 0.0, -0.4, 0.0, 0.8]  # Open grip
        self.send_pose(empty_hand, 2.0)
        
        # Ta-da!
        self.send_pose("ta_da", 2.0)
    
    def card_trick(self):
        """Perform a card trick"""
        rospy.loginfo("🃏 Now for a classic... pick a card, any card!")
        
        # Fan out cards
        rospy.loginfo("🎴 Fanning out the deck...")
        fan_left = [-0.4, -0.2, 0.7, 0.0, -0.5, 0.0, 0.3]
        fan_right = [0.4, -0.2, 0.7, 0.0, -0.5, 0.0, 0.3]
        
        for _ in range(2):
            self.send_pose(fan_left, 1.0)
            self.send_pose(fan_right, 1.0)
        
        # Center position
        self.send_pose([0.0, -0.2, 0.7, 0.0, -0.5, 0.0, 0.3], 1.0)
        
        # Pretend someone picks a card
        rospy.loginfo("👆 Ah, excellent choice! Remember your card...")
        time.sleep(2.0)
        
        # Shuffle cards with flair
        rospy.loginfo("🔀 Now I'll shuffle the deck...")
        for shuffle in range(4):
            shuffle_high = [0.0, 0.1, 0.4, 0.0, 0.2, 0.3, 0.3]
            shuffle_low = [0.0, -0.3, 0.8, 0.0, -0.6, -0.3, 0.3]
            self.send_pose(shuffle_high, 0.4)
            self.send_pose(shuffle_low, 0.4)
        
        # Magic gesture
        rospy.loginfo("✨ With a little magic...")
        self.send_pose("wand_flourish", 2.0)
        
        # Dramatic reveal
        rospy.loginfo("🎯 Is THIS your card?!")
        reveal_card = [0.0, 0.2, 0.4, 0.0, 0.1, 0.0, 0.2]
        self.send_pose(reveal_card, 2.0)
        
        # Audience amazement pause
        time.sleep(2.0)
        rospy.loginfo("👏 *Audience gasps in amazement*")
    
    def levitation_trick(self):
        """Simulate levitation"""
        rospy.loginfo("🪶 Prepare to witness... LEVITATION!")
        
        # Place object on 'table'
        rospy.loginfo("📦 Placing object on the table...")
        place_object = [0.0, -0.4, 1.0, 0.0, -0.6, 0.0, 0.8]
        self.send_pose(place_object, 2.0)
        
        # Step back and prepare
        self.send_pose("mysterious", 2.0)
        
        # Concentration gesture
        rospy.loginfo("🧘 Concentrating my mystical powers...")
        concentrate = [0.0, -0.1, 0.6, 0.0, -0.5, 0.0, 0.5]
        self.send_pose(concentrate, 3.0)
        
        # Levitation motion - slowly raise hands
        rospy.loginfo("⬆️ Rise... RISE!")
        
        for height in [0.0, 0.1, 0.2, 0.3, 0.4]:
            levitate_pose = [0.0, -0.1 + height, 0.6 - height*0.5, 0.0, -0.5 + height, 0.0, 0.8]
            self.send_pose(levitate_pose, 1.5)
            rospy.loginfo(f"✨ Object rising... {int(height*100)}% levitated!")
        
        # Hold levitation
        rospy.loginfo("🌟 BEHOLD! The object floats in mid-air!")
        time.sleep(3.0)
        
        # Slowly lower
        rospy.loginfo("⬇️ Gently returning to earth...")
        for height in [0.3, 0.2, 0.1, 0.0]:
            lower_pose = [0.0, -0.1 + height, 0.6 - height*0.5, 0.0, -0.5 + height, 0.0, 0.8]
            self.send_pose(lower_pose, 1.5)
        
        # Final flourish
        self.send_pose("ta_da", 2.0)
    
    def rabbit_from_hat(self):
        """Pull a rabbit from a hat"""
        rospy.loginfo("🎩 For my final trick... something from nothing!")
        
        # Show empty hat
        rospy.loginfo("👀 As you can see, the hat is completely empty...")
        show_hat = [0.0, -0.2, 0.6, 0.0, -0.4, 0.0, 0.8]
        self.send_pose(show_hat, 2.0)
        
        # Tip hat to show inside
        tip_hat = [0.2, -0.1, 0.5, -0.3, -0.2, 0.0, 0.8]
        self.send_pose(tip_hat, 1.5)
        self.send_pose(show_hat, 1.5)
        
        # Cover hat with cloth (other hand)
        rospy.loginfo("🧙 Covering with my magic cloth...")
        cover_motion = [0.3, 0.0, 0.4, 0.0, 0.1, 0.0, 0.8]
        self.send_pose(cover_motion, 2.0)
        
        # Magic incantation with dramatic gestures
        rospy.loginfo("🪄 Hocus Pocus! Sim Sala Bim!")
        
        for _ in range(3):
            magic_wave = [0.0, 0.4, 0.2, 0.0, 0.6, 1.0, 0.8]
            magic_point = [0.0, -0.2, 0.8, 0.0, -0.6, -1.0, 0.8]
            self.send_pose(magic_wave, 0.8)
            self.send_pose(magic_point, 0.8)
        
        # Dramatic pause
        self.send_pose("mysterious", 2.0)
        time.sleep(2.0)
        
        # Reach into hat
        rospy.loginfo("🐰 Reaching into the hat...")
        reach_in = [0.0, -0.3, 0.9, 0.0, -0.6, 0.0, 0.8]
        self.send_pose(reach_in, 2.0)
        
        # Grab something
        grab_rabbit = [0.0, -0.3, 0.9, 0.0, -0.6, 0.0, 0.2]
        self.send_pose(grab_rabbit, 1.0)
        
        # Slowly pull out
        rospy.loginfo("🎉 VOILA! A rabbit!")
        
        pull_stages = [
            [0.0, -0.2, 0.8, 0.0, -0.5, 0.0, 0.2],
            [0.0, -0.1, 0.6, 0.0, -0.3, 0.0, 0.2],
            [0.0, 0.0, 0.4, 0.0, -0.1, 0.0, 0.2],
            [0.0, 0.1, 0.3, 0.0, 0.1, 0.0, 0.2]
        ]
        
        for stage in pull_stages:
            self.send_pose(stage, 1.0)
            rospy.loginfo("🐰 *rabbit ears appearing*")
        
        # Present the rabbit
        rospy.loginfo("🎊 TADA! Ladies and gentlemen... the rabbit!")
        present_rabbit = [0.0, 0.2, 0.3, 0.0, 0.2, 0.0, 0.2]
        self.send_pose(present_rabbit, 2.0)
        
        # Bow with rabbit
        time.sleep(2.0)
    
    def grand_finale(self):
        """Grand finale with multiple effects"""
        rospy.loginfo("🎆 And now... the GRAND FINALE!")
        
        # Build up suspense
        self.send_pose("mysterious", 2.0)
        
        # Dramatic countdown
        for count in [3, 2, 1]:
            rospy.loginfo(f"⏰ {count}...")
            dramatic_pose = [0.0, 0.3 * count/3, 0.4, 0.0, 0.2 * count/3, 0.0, 0.8]
            self.send_pose(dramatic_pose, 1.0)
        
        # EXPLOSION of magic
        rospy.loginfo("💥 ABRACADABRA!")
        
        # Rapid fire magical gestures
        finale_moves = [
            [0.5, 0.5, 0.1, 0.0, 0.6, 1.2, 0.8],
            [-0.5, 0.5, 0.1, 0.0, 0.6, -1.2, 0.8],
            [0.0, 0.8, -0.1, 0.0, 0.7, 0.0, 0.8],
            [0.0, -0.2, 1.2, 0.0, -0.8, 0.0, 0.8]
        ]
        
        for move in finale_moves:
            self.send_pose(move, 0.5)
        
        # Final bow
        rospy.loginfo("🎭 Thank you, thank you! Z1 the Magnificent!")
        final_bow = [0.0, -0.8, 1.5, 0.0, -0.7, 0.0, 0.0]
        self.send_pose(final_bow, 3.0)
        
        # Tip hat one last time
        self.send_pose("hat_tip", 2.0)
    
    def magic_show(self):
        """Complete magic show performance"""
        rospy.loginfo("🎪 Welcome to Z1's Amazing Magic Show!")
        
        self.magic_introduction()
        time.sleep(1.0)
        
        tricks = [
            ("The Disappearing Act", self.disappearing_act),
            ("Mind-Reading Card Trick", self.card_trick),
            ("Mystical Levitation", self.levitation_trick),
            ("Rabbit from Hat", self.rabbit_from_hat)
        ]
        
        for trick_name, trick_func in tricks:
            rospy.loginfo(f"\\n🎭 {trick_name}")
            trick_func()
            time.sleep(2.0)
            rospy.loginfo("👏 *Thunderous applause*")
            time.sleep(1.0)
        
        self.grand_finale()
    
    def return_home(self):
        """Return to neutral position"""
        rospy.loginfo("🎩 The magic show has ended... or has it?")
        self.send_pose([0.0] * 7, 3.0)

def main():
    parser = argparse.ArgumentParser(description="Z1 Master Magician")
    parser.add_argument("--mode", type=int, default=10)
    parser.add_argument("--kp", type=float, default=35.0)
    parser.add_argument("--kd", type=float, default=1.5)
    parser.add_argument("--trick", choices=["disappear", "cards", "levitation", "rabbit", "show"],
                       default="show", help="Specific trick or full show")
    args = parser.parse_args()
    
    try:
        magician = Z1Magician(args.mode, args.kp, args.kd)
        
        if args.trick == "disappear":
            magician.disappearing_act()
        elif args.trick == "cards":
            magician.card_trick()
        elif args.trick == "levitation":
            magician.levitation_trick()
        elif args.trick == "rabbit":
            magician.rabbit_from_hat()
        else:  # show
            magician.magic_show()
        
        magician.return_home()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Magic show interrupted")

if __name__ == "__main__":
    main()