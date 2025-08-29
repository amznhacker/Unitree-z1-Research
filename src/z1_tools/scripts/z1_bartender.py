#!/usr/bin/env python3
"""
Z1 Bartender
Mix cocktails with flair and precision
"""
import rospy, math, time, random, argparse
from unitree_legged_msgs.msg import MotorCmd

class Z1Bartender:
    def __init__(self, mode=10, kp=35.0, kd=1.5):
        rospy.init_node("z1_bartender")
        
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
        
        # Bar positions
        self.bar_positions = {
            "bottle_shelf": [0.5, -0.1, 0.4, 0.0, -0.3, 0.0, 0.8],
            "shaker_grab": [0.0, -0.3, 0.8, 0.0, -0.5, 0.0, 0.8],
            "mixing_high": [0.0, 0.2, 0.5, 0.0, 0.3, 0.0, 0.3],
            "mixing_low": [0.0, -0.4, 1.0, 0.0, -0.6, 0.0, 0.3],
            "pour_position": [-0.3, -0.2, 0.6, -0.5, -0.4, 0.0, 0.3],
            "garnish_station": [0.4, -0.2, 0.7, 0.0, -0.5, 0.0, 0.5],
            "serve_position": [0.0, -0.1, 0.3, 0.0, -0.2, 0.0, 0.8]
        }
        
        # Cocktail recipes
        self.cocktails = {
            "martini": ["gin", "vermouth", "olive"],
            "mojito": ["rum", "mint", "lime", "soda"],
            "margarita": ["tequila", "lime", "salt"],
            "old_fashioned": ["whiskey", "sugar", "bitters", "orange"]
        }
    
    def send_pose(self, pose_name_or_positions, duration=2.0):
        """Move to pose smoothly"""
        if isinstance(pose_name_or_positions, str):
            if pose_name_or_positions in self.bar_positions:
                target = self.bar_positions[pose_name_or_positions]
            else:
                rospy.logerr(f"Unknown position: {pose_name_or_positions}")
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
    
    def grab_bottle(self, bottle_name):
        """Grab a bottle from the shelf"""
        rospy.loginfo(f"🍾 Grabbing {bottle_name} bottle...")
        
        # Reach for bottle
        self.send_pose("bottle_shelf", 2.0)
        
        # Close gripper on bottle
        grab_pose = self.bar_positions["bottle_shelf"].copy()
        grab_pose[6] = 0.2  # Close grip
        self.send_pose(grab_pose, 0.5)
        
        # Lift bottle with flair
        lift_pose = grab_pose.copy()
        lift_pose[1] += 0.3  # Lift up
        self.send_pose(lift_pose, 1.0)
    
    def bottle_flip(self):
        """Perform a flashy bottle flip"""
        rospy.loginfo("🤹 Performing bottle flip!")
        
        # Quick wrist flips
        for _ in range(3):
            flip_up = [0.0, 0.0, 0.6, 0.0, 0.8, 1.2, 0.2]
            flip_down = [0.0, 0.0, 0.6, 0.0, -0.8, -1.2, 0.2]
            
            self.send_pose(flip_up, 0.3)
            self.send_pose(flip_down, 0.3)
        
        # Catch in normal position
        self.send_pose([0.0, -0.1, 0.5, 0.0, -0.4, 0.0, 0.2], 1.0)
    
    def pour_ingredient(self, ingredient, duration=2.0):
        """Pour ingredient with style"""
        rospy.loginfo(f"🥃 Pouring {ingredient}...")
        
        # Move to pour position
        self.send_pose("pour_position", 1.5)
        
        # Pouring motion with wrist rotation
        start_time = time.time()
        base_pose = self.bar_positions["pour_position"]
        
        while time.time() - start_time < duration:
            t = time.time() - start_time
            
            # Gentle pouring motion
            pour_angle = 0.3 * math.sin(t * 2)  # Gentle tilt
            
            pour_pose = base_pose.copy()
            pour_pose[4] += pour_angle  # Wrist tilt for pouring
            pour_pose[5] = 0.1 * math.sin(t * 4)  # Slight wrist roll
            
            self.send_pose(pour_pose, 0.1)
        
        # Return bottle upright
        self.send_pose(base_pose, 0.5)
    
    def shake_cocktail(self, intensity=3):
        """Shake cocktail with rhythm"""
        rospy.loginfo(f"🍸 Shaking cocktail with intensity {intensity}!")
        
        # Grab shaker
        self.send_pose("shaker_grab", 2.0)
        
        # Close grip on shaker
        shake_pose = self.bar_positions["shaker_grab"].copy()
        shake_pose[6] = 0.1  # Firm grip
        self.send_pose(shake_pose, 0.5)
        
        # Rhythmic shaking motion
        for shake_round in range(intensity):
            rospy.loginfo(f"Shake round {shake_round + 1}")
            
            # Fast up-down shaking
            for _ in range(8):
                # Up position
                self.send_pose("mixing_high", 0.2)
                # Down position  
                self.send_pose("mixing_low", 0.2)
            
            # Brief pause between rounds
            time.sleep(0.3)
        
        # Final flourish
        self.send_pose("mixing_high", 1.0)
        rospy.loginfo("🎵 Shaking complete - perfect mix achieved!")
    
    def add_garnish(self, garnish):
        """Add garnish with precision"""
        rospy.loginfo(f"🍋 Adding {garnish} garnish...")
        
        # Move to garnish station
        self.send_pose("garnish_station", 2.0)
        
        # Pick up garnish
        garnish_pose = self.bar_positions["garnish_station"].copy()
        garnish_pose[6] = 0.3  # Delicate grip
        self.send_pose(garnish_pose, 1.0)
        
        # Precise placement motion
        place_garnish = [0.0, -0.1, 0.4, 0.0, -0.3, 0.0, 0.3]
        self.send_pose(place_garnish, 2.0)
        
        # Release garnish
        place_garnish[6] = 0.8
        self.send_pose(place_garnish, 0.5)
    
    def serve_drink(self):
        """Present the finished cocktail"""
        rospy.loginfo("🍹 Presenting your cocktail!")
        
        # Move to serve position
        self.send_pose("serve_position", 2.0)
        
        # Elegant presentation gesture
        for _ in range(2):
            present_high = [0.0, 0.1, 0.2, 0.0, 0.0, 0.0, 0.8]
            self.send_pose(present_high, 1.0)
            self.send_pose("serve_position", 1.0)
        
        # Bow gesture
        bow_pose = [0.0, -0.5, 1.2, 0.0, -0.7, 0.0, 0.8]
        self.send_pose(bow_pose, 1.5)
        self.send_pose("serve_position", 1.5)
    
    def mix_cocktail(self, cocktail_name):
        """Mix a complete cocktail"""
        if cocktail_name not in self.cocktails:
            rospy.logerr(f"Unknown cocktail: {cocktail_name}")
            return
        
        ingredients = self.cocktails[cocktail_name]
        rospy.loginfo(f"🍸 Mixing {cocktail_name.title()} with {len(ingredients)} ingredients")
        
        # Prepare workspace
        rospy.loginfo("🧹 Preparing bar workspace...")
        self.send_pose([0.0, -0.3, 0.8, 0.0, -0.5, 0.0, 0.8], 2.0)
        
        # Add each ingredient
        for i, ingredient in enumerate(ingredients[:-1]):  # Save garnish for last
            self.grab_bottle(ingredient)
            
            # Add some flair for premium ingredients
            if ingredient in ["gin", "whiskey", "tequila"]:
                self.bottle_flip()
            
            self.pour_ingredient(ingredient, duration=1.5 + i * 0.5)
            
            # Return bottle
            self.send_pose("bottle_shelf", 1.5)
            time.sleep(0.5)
        
        # Shake if needed
        if cocktail_name in ["martini", "margarita"]:
            self.shake_cocktail(intensity=3)
        elif cocktail_name == "mojito":
            rospy.loginfo("🌿 Muddling mint leaves...")
            # Muddling motion
            for _ in range(5):
                muddle_down = [0.0, -0.4, 1.2, 0.0, -0.8, 0.0, 0.1]
                muddle_up = [0.0, -0.2, 0.8, 0.0, -0.6, 0.0, 0.1]
                self.send_pose(muddle_down, 0.3)
                self.send_pose(muddle_up, 0.3)
        
        # Add garnish
        if len(ingredients) > 0:
            self.add_garnish(ingredients[-1])
        
        # Serve with style
        self.serve_drink()
        
        rospy.loginfo(f"✨ {cocktail_name.title()} is ready! Enjoy responsibly! 🍻")
    
    def bartender_show(self):
        """Full bartender performance"""
        cocktails_to_make = ["martini", "mojito", "margarita"]
        
        rospy.loginfo("🎭 Welcome to Z1's Cocktail Show!")
        
        for cocktail in cocktails_to_make:
            rospy.loginfo(f"\\n🎪 Now featuring: {cocktail.title()}!")
            self.mix_cocktail(cocktail)
            time.sleep(2.0)
        
        # Final bow
        rospy.loginfo("🎉 Thank you for visiting Z1's Bar!")
        final_bow = [0.0, -0.8, 1.5, 0.0, -0.7, 0.0, 0.0]
        self.send_pose(final_bow, 3.0)
    
    def return_home(self):
        """Clean up and return to rest"""
        rospy.loginfo("🧽 Cleaning up the bar...")
        self.send_pose([0.0] * 7, 3.0)

def main():
    parser = argparse.ArgumentParser(description="Z1 Master Bartender")
    parser.add_argument("--mode", type=int, default=10)
    parser.add_argument("--kp", type=float, default=35.0)
    parser.add_argument("--kd", type=float, default=1.5)
    parser.add_argument("--cocktail", choices=["martini", "mojito", "margarita", "old_fashioned", "show"],
                       default="show", help="Cocktail to make or full show")
    args = parser.parse_args()
    
    try:
        bartender = Z1Bartender(args.mode, args.kp, args.kd)
        
        if args.cocktail == "show":
            bartender.bartender_show()
        else:
            bartender.mix_cocktail(args.cocktail)
        
        bartender.return_home()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Bar service interrupted")

if __name__ == "__main__":
    main()