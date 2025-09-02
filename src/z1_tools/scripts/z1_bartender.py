#!/usr/bin/env python3
"""
Z1 Professional Bartender
Realistic bartender that picks up bottles and tools from table to mix cocktails
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
        
        # Table positions for bottles and tools (realistic bar setup)
        self.table_positions = {
            # Bottles on table (left to right)
            "gin_bottle": [-0.4, -0.3, 1.0, 0.0, -0.7, 0.0, 0.8],
            "vodka_bottle": [-0.2, -0.3, 1.0, 0.0, -0.7, 0.0, 0.8], 
            "rum_bottle": [0.0, -0.3, 1.0, 0.0, -0.7, 0.0, 0.8],
            "tequila_bottle": [0.2, -0.3, 1.0, 0.0, -0.7, 0.0, 0.8],
            "whiskey_bottle": [0.4, -0.3, 1.0, 0.0, -0.7, 0.0, 0.8],
            
            # Bar tools
            "shaker": [-0.3, -0.2, 0.8, 0.0, -0.6, 0.0, 0.8],
            "jigger": [-0.1, -0.2, 0.8, 0.0, -0.6, 0.0, 0.8],
            "strainer": [0.1, -0.2, 0.8, 0.0, -0.6, 0.0, 0.8],
            "muddler": [0.3, -0.2, 0.8, 0.0, -0.6, 0.0, 0.8],
            
            # Garnish tray
            "lime_wedge": [-0.4, -0.1, 0.6, 0.0, -0.5, 0.0, 0.8],
            "lemon_twist": [-0.2, -0.1, 0.6, 0.0, -0.5, 0.0, 0.8],
            "olive": [0.0, -0.1, 0.6, 0.0, -0.5, 0.0, 0.8],
            "mint_sprig": [0.2, -0.1, 0.6, 0.0, -0.5, 0.0, 0.8],
            
            # Glass positions
            "mixing_glass": [0.0, -0.4, 1.2, 0.0, -0.8, 0.0, 0.8],
            "serving_glass": [0.0, -0.1, 0.4, 0.0, -0.3, 0.0, 0.8],
            
            # Working positions
            "shake_high": [0.0, 0.3, 0.4, 0.0, 0.4, 0.0, 0.2],
            "shake_low": [0.0, -0.5, 1.2, 0.0, -0.8, 0.0, 0.2],
            "pour_over_glass": [0.0, -0.2, 0.6, -0.3, -0.4, 0.0, 0.2]
        }
        
        # Cocktail recipes with actual table items
        self.cocktail_recipes = {
            "martini": {
                "base": "gin_bottle",
                "tools": ["jigger", "shaker", "strainer"],
                "garnish": "olive",
                "method": "shake"
            },
            "old_fashioned": {
                "base": "whiskey_bottle", 
                "tools": ["jigger", "muddler"],
                "garnish": "lemon_twist",
                "method": "stir"
            },
            "mojito": {
                "base": "rum_bottle",
                "tools": ["muddler", "jigger"],
                "garnish": "mint_sprig",
                "method": "muddle"
            }
        }
    
    def send_pose(self, pose_name_or_positions, duration=2.0):
        """Move to pose smoothly"""
        if isinstance(pose_name_or_positions, str):
            if pose_name_or_positions in self.table_positions:
                target = self.table_positions[pose_name_or_positions]
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
    
    def pick_up_item(self, item_name):
        """Pick up an item from the table"""
        rospy.loginfo(f"🤏 Picking up {item_name.replace('_', ' ')}...")
        
        if item_name not in self.table_positions:
            rospy.logerr(f"Item {item_name} not found on table!")
            return False
        
        # Approach item position
        approach_pos = self.table_positions[item_name].copy()
        approach_pos[1] += 0.1  # Slightly above item
        approach_pos[6] = 0.8   # Open gripper
        self.send_pose(approach_pos, 2.0)
        
        # Lower to item
        grab_pos = self.table_positions[item_name].copy()
        grab_pos[6] = 0.8  # Still open
        self.send_pose(grab_pos, 1.0)
        
        # Close gripper to grab
        grab_pos[6] = 0.2  # Close grip
        self.send_pose(grab_pos, 0.5)
        rospy.loginfo(f"✅ Grabbed {item_name.replace('_', ' ')}")
        
        # Lift item
        lift_pos = grab_pos.copy()
        lift_pos[1] += 0.2  # Lift up
        self.send_pose(lift_pos, 1.0)
        
        return True
    
    def bottle_flip(self):
        """Perform a flashy bottle flip"""
        rospy.loginfo("🤹 Performing bottle flip!")
        
        # Quick wrist flips
        for _ in range(2):
            flip_up = [0.0, 0.0, 0.6, 0.0, 0.8, 1.2, 0.2]
            flip_down = [0.0, 0.0, 0.6, 0.0, -0.8, -1.2, 0.2]
            
            self.send_pose(flip_up, 0.4)
            self.send_pose(flip_down, 0.4)
        
        # Catch in normal position
        self.send_pose([0.0, -0.1, 0.5, 0.0, -0.4, 0.0, 0.2], 1.0)
    
    def pour_into_glass(self, duration=2.0):
        """Pour from held bottle into mixing glass"""
        rospy.loginfo("🥃 Pouring into glass...")
        
        # Move to pour position over glass
        self.send_pose("pour_over_glass", 1.5)
        
        # Pouring motion with wrist tilt
        start_time = time.time()
        base_pose = self.table_positions["pour_over_glass"]
        
        while time.time() - start_time < duration:
            t = time.time() - start_time
            progress = t / duration
            
            # Gradual tilt for pouring
            pour_angle = 0.4 * progress  # Increase tilt over time
            
            pour_pose = base_pose.copy()
            pour_pose[4] += pour_angle  # Wrist tilt for pouring
            
            self.send_pose(pour_pose, 0.1)
        
        # Return bottle upright
        self.send_pose(base_pose, 0.5)
        rospy.loginfo("✅ Poured successfully")
    
    def shake_with_shaker(self, intensity=3):
        """Shake cocktail using the shaker from table"""
        rospy.loginfo(f"🍸 Shaking cocktail with intensity {intensity}!")
        
        # Rhythmic shaking motion (shaker already in hand)
        for shake_round in range(intensity):
            rospy.loginfo(f"Shake round {shake_round + 1}")
            
            # Fast up-down shaking
            for _ in range(6):
                # Up position
                self.send_pose("shake_high", 0.25)
                # Down position  
                self.send_pose("shake_low", 0.25)
            
            # Brief pause between rounds
            time.sleep(0.2)
        
        # Final flourish
        self.send_pose("shake_high", 1.0)
        rospy.loginfo("🎵 Shaking complete - perfect mix achieved!")
    
    def place_item_down(self, target_position="serving_glass"):
        """Place held item down at target position"""
        rospy.loginfo(f"📍 Placing item at {target_position.replace('_', ' ')}...")
        
        # Move to target position
        place_pos = self.table_positions[target_position].copy()
        place_pos[1] += 0.1  # Slightly above target
        self.send_pose(place_pos, 2.0)
        
        # Lower to place
        place_pos[1] -= 0.1  # Down to surface
        self.send_pose(place_pos, 1.0)
        
        # Release item
        place_pos[6] = 0.8  # Open gripper
        self.send_pose(place_pos, 0.5)
        
        # Move away
        place_pos[1] += 0.1  # Lift up
        self.send_pose(place_pos, 1.0)
        
        rospy.loginfo("✅ Item placed successfully")
    
    def present_cocktail(self):
        """Present the finished cocktail"""
        rospy.loginfo("🍹 Presenting your cocktail!")
        
        # Pick up finished drink
        self.pick_up_item("serving_glass")
        
        # Elegant presentation gesture
        present_pose = [0.0, 0.2, 0.3, 0.0, 0.1, 0.0, 0.2]
        self.send_pose(present_pose, 2.0)
        
        # Hold for admiration
        time.sleep(2.0)
        
        # Place back down
        self.place_item_down("serving_glass")
        
        # Bartender bow
        bow_pose = [0.0, -0.6, 1.4, 0.0, -0.8, 0.0, 0.0]
        self.send_pose(bow_pose, 2.0)
        
        # Return to neutral
        self.send_pose([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8], 2.0)
    
    def mix_cocktail(self, cocktail_name):
        """Mix a complete cocktail using items from table"""
        if cocktail_name not in self.cocktail_recipes:
            rospy.logerr(f"Unknown cocktail: {cocktail_name}")
            return
        
        recipe = self.cocktail_recipes[cocktail_name]
        rospy.loginfo(f"🍸 Mixing {cocktail_name.title()} - professional bartender style!")
        
        # Step 1: Prepare mixing glass
        rospy.loginfo("🥃 Preparing mixing glass...")
        self.pick_up_item("mixing_glass")
        self.place_item_down("mixing_glass")  # Ensure it's in position
        
        # Step 2: Get the base spirit
        rospy.loginfo(f"🍾 Getting {recipe['base'].replace('_', ' ')}...")
        self.pick_up_item(recipe['base'])
        
        # Add some flair for premium spirits
        if "whiskey" in recipe['base'] or "gin" in recipe['base']:
            self.bottle_flip()
        
        # Pour base spirit
        self.pour_into_glass(duration=2.0)
        self.place_item_down(recipe['base'])  # Put bottle back
        
        # Step 3: Use appropriate method
        if recipe['method'] == 'shake':
            rospy.loginfo("🥤 Time to shake!")
            self.pick_up_item("shaker")
            self.shake_with_shaker(intensity=3)
            self.place_item_down("shaker")
            
        elif recipe['method'] == 'muddle':
            rospy.loginfo("🌿 Muddling ingredients...")
            self.pick_up_item("muddler")
            
            # Muddling motion in glass
            muddle_pos = self.table_positions["mixing_glass"].copy()
            for _ in range(6):
                muddle_pos[1] -= 0.05  # Press down
                self.send_pose(muddle_pos, 0.3)
                muddle_pos[1] += 0.05  # Lift up
                self.send_pose(muddle_pos, 0.3)
            
            self.place_item_down("muddler")
            
        elif recipe['method'] == 'stir':
            rospy.loginfo("🥄 Stirring with precision...")
            # Stirring motion over glass
            stir_pos = self.table_positions["mixing_glass"].copy()
            stir_pos[1] += 0.05  # Slightly above glass
            
            for _ in range(8):
                # Circular stirring motion
                for angle in [0, 90, 180, 270]:
                    stir_pos[0] = 0.05 * math.cos(math.radians(angle))
                    stir_pos[5] = 0.1 * math.sin(math.radians(angle))
                    self.send_pose(stir_pos, 0.2)
        
        # Step 4: Transfer to serving glass
        rospy.loginfo("🍸 Transferring to serving glass...")
        self.pick_up_item("mixing_glass")
        
        # Pour into serving glass
        serve_pour_pos = self.table_positions["serving_glass"].copy()
        serve_pour_pos[1] += 0.1  # Above serving glass
        serve_pour_pos[4] = -0.3  # Tilt for pouring
        self.send_pose(serve_pour_pos, 2.0)
        
        # Pour motion
        time.sleep(2.0)
        
        # Return mixing glass
        self.place_item_down("mixing_glass")
        
        # Step 5: Add garnish
        rospy.loginfo(f"🍋 Adding {recipe['garnish'].replace('_', ' ')} garnish...")
        self.pick_up_item(recipe['garnish'])
        
        # Place garnish on drink
        garnish_pos = self.table_positions["serving_glass"].copy()
        garnish_pos[1] += 0.05  # Just above glass
        self.send_pose(garnish_pos, 1.5)
        
        # Drop garnish
        garnish_pos[6] = 0.8  # Open gripper
        self.send_pose(garnish_pos, 0.5)
        
        # Step 6: Present the masterpiece
        self.present_cocktail()
        
        rospy.loginfo(f"✨ {cocktail_name.title()} is ready! Crafted with precision! 🍻")
    
    def bartender_show(self):
        """Full bartender performance with table setup"""
        cocktails_to_make = ["old_fashioned", "martini", "mojito"]
        
        rospy.loginfo("🎭 Welcome to Z1's Professional Bar!")
        rospy.loginfo("📋 Table Setup: Bottles, tools, and garnishes are ready")
        
        # Bartender introduction
        intro_pose = [0.0, 0.1, 0.4, 0.0, 0.2, 0.0, 0.8]
        self.send_pose(intro_pose, 2.0)
        
        for i, cocktail in enumerate(cocktails_to_make):
            rospy.loginfo(f"\\n🎪 Cocktail #{i+1}: {cocktail.title().replace('_', ' ')}!")
            self.mix_cocktail(cocktail)
            
            if i < len(cocktails_to_make) - 1:
                rospy.loginfo("🧹 Cleaning workspace for next cocktail...")
                time.sleep(2.0)
        
        # Final bow
        rospy.loginfo("🎉 Thank you for visiting Z1's Professional Bar!")
        rospy.loginfo("🍻 All cocktails crafted with precision and flair!")
        final_bow = [0.0, -0.8, 1.5, 0.0, -0.7, 0.0, 0.0]
        self.send_pose(final_bow, 3.0)
    
    def return_home(self):
        """Clean up and return to rest"""
        rospy.loginfo("🧽 Cleaning up the bar...")
        self.send_pose([0.0] * 7, 3.0)

def main():
    parser = argparse.ArgumentParser(description="Z1 Professional Bartender")
    parser.add_argument("--mode", type=int, default=10)
    parser.add_argument("--kp", type=float, default=35.0)
    parser.add_argument("--kd", type=float, default=1.5)
    parser.add_argument("--cocktail", choices=["martini", "mojito", "old_fashioned", "show"],
                       default="show", help="Cocktail to make or full show")
    parser.add_argument("--setup", action="store_true", help="Show table setup instructions")
    args = parser.parse_args()
    
    if args.setup:
        print("🍸 Z1 Bartender Table Setup Instructions:")
        print("==========================================")
        print("📍 Place items on table in front of Z1:")
        print("\\n🍾 Bottles (left to right):")
        print("  - Gin bottle (left side)")
        print("  - Vodka bottle")
        print("  - Rum bottle (center)")
        print("  - Tequila bottle")
        print("  - Whiskey bottle (right side)")
        print("\\n🛠️ Bar Tools (middle row):")
        print("  - Cocktail shaker")
        print("  - Jigger (measuring cup)")
        print("  - Strainer")
        print("  - Muddler")
        print("\\n🍋 Garnishes (front row):")
        print("  - Lime wedges")
        print("  - Lemon twists")
        print("  - Olives")
        print("  - Mint sprigs")
        print("\\n🥃 Glasses:")
        print("  - Mixing glass (center back)")
        print("  - Serving glass (center front)")
        print("\\n✨ Ready to bartend!")
        return
    
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