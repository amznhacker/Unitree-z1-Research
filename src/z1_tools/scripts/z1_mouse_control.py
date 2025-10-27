#!/usr/bin/env python3

"""
Z1 Enhanced Mouse Control - Full 6-DOF + gripper control
Mouse X/Y + modifiers = All joints, Clicks = Gripper, Scroll = Mode switching
"""

import rospy
import sys
from unitree_legged_msgs.msg import MotorCmd

try:
    import pygame
except ImportError:
    print("Error: pygame not installed. Run: pip3 install pygame")
    sys.exit(1)

class Z1MouseControl:
    def __init__(self):
        rospy.init_node("z1_mouse_control")
        
        # Initialize pygame
        pygame.init()
        self.screen = pygame.display.set_mode((600, 400))
        pygame.display.set_caption("Z1 Full Mouse Control - Hold keys for different joints")
        
        # Joint limits and positions
        self.limits = {
            "Joint01": (-1.2, 1.2),   # Base ±69°
            "Joint02": (-1.0, 1.0),   # Shoulder ±57°
            "Joint03": (0.0, 2.4),    # Elbow 0° to 137°
            "Joint04": (-1.2, 1.2),   # Forearm ±69°
            "Joint05": (-1.0, 1.0),   # Wrist pitch ±57°
            "Joint06": (-1.2, 1.2),   # Wrist roll ±69°
            "Gripper": (0.0, 0.6)     # Gripper 0-60%
        }
        self.positions = {j: 0.0 for j in self.limits.keys()}
        
        # Control settings
        self.sensitivity = 0.003
        self.click_step = 0.1
        self.running = True
        self.control_mode = "base_shoulder"  # Current control mode
        
        # Control modes
        self.modes = {
            "base_shoulder": ["Joint01", "Joint02"],
            "elbow_forearm": ["Joint03", "Joint04"],
            "wrist_pitch_roll": ["Joint05", "Joint06"]
        }
        self.mode_index = 0
        
        # Publishers
        self.pubs = {}
        for joint in self.limits.keys():
            controller = f"{joint}_controller"
            topic = f"/z1_gazebo/{controller}/command"
            self.pubs[joint] = rospy.Publisher(topic, MotorCmd, queue_size=1)
        
        rospy.loginfo("Z1 Enhanced Mouse Control initialized")
        
    def clamp(self, joint, pos):
        min_pos, max_pos = self.limits[joint]
        return max(min_pos, min(max_pos, pos))
    
    def set_joint(self, joint, pos):
        self.positions[joint] = self.clamp(joint, pos)
        
        msg = MotorCmd()
        msg.mode = 10
        msg.q = float(self.positions[joint])
        msg.dq = 0.0
        msg.tau = 0.0
        msg.Kp = 35.0
        msg.Kd = 1.5
        
        self.pubs[joint].publish(msg)
    
    def stop_all(self):
        rospy.logwarn("STOPPING - Returning to neutral")
        for joint in self.positions.keys():
            self.set_joint(joint, 0.0)
    
    def get_current_joints(self):
        """Get current control joints based on mode"""
        return self.modes[list(self.modes.keys())[self.mode_index]]
    
    def switch_mode(self):
        """Switch to next control mode"""
        self.mode_index = (self.mode_index + 1) % len(self.modes)
        mode_name = list(self.modes.keys())[self.mode_index]
        joints = self.modes[mode_name]
        print(f"\nMode: {mode_name.replace('_', ' ').title()} - Controlling {joints[0]} (X) & {joints[1]} (Y)")
    
    def draw_interface(self):
        """Draw control interface"""
        self.screen.fill((20, 20, 30))
        
        # Create font
        font = pygame.font.Font(None, 24)
        small_font = pygame.font.Font(None, 18)
        
        # Current mode
        mode_name = list(self.modes.keys())[self.mode_index]
        joints = self.get_current_joints()
        mode_text = font.render(f"Mode: {mode_name.replace('_', ' ').title()}", True, (255, 255, 255))
        self.screen.blit(mode_text, (10, 10))
        
        control_text = small_font.render(f"Mouse X/Y controls {joints[0]} & {joints[1]}", True, (200, 200, 200))
        self.screen.blit(control_text, (10, 35))
        
        # Joint positions
        y_pos = 70
        for joint, pos in self.positions.items():
            color = (0, 255, 0) if joint in joints else (150, 150, 150)
            text = small_font.render(f"{joint}: {pos:.3f}", True, color)
            self.screen.blit(text, (10, y_pos))
            y_pos += 20
        
        # Controls help
        help_y = 250
        help_texts = [
            "Scroll: Switch control mode",
            "Left Click: Open gripper", 
            "Right Click: Close gripper",
            "ESC: Emergency stop"
        ]
        for text in help_texts:
            help_text = small_font.render(text, True, (180, 180, 180))
            self.screen.blit(help_text, (10, help_y))
            help_y += 18
    
    def run(self):
        print("Z1 Enhanced Mouse Control Started")
        print("Full 6-DOF + Gripper Control:")
        print("  Scroll Wheel: Switch control modes")
        print("  Mouse X/Y: Control active joint pair")
        print("  Left Click: Open gripper")
        print("  Right Click: Close gripper")
        print("  ESC: Emergency stop")
        
        clock = pygame.time.Clock()
        center_x, center_y = 300, 200
        last_mouse_pos = (center_x, center_y)
        
        # Initialize mode
        self.switch_mode()
        
        try:
            while self.running and not rospy.is_shutdown():
                for event in pygame.event.get():
                    if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                        self.running = False
                    elif event.type == pygame.MOUSEBUTTONDOWN:
                        if event.button == 1:  # Left click - open gripper
                            new_pos = min(0.6, self.positions["Gripper"] + self.click_step)
                            self.set_joint("Gripper", new_pos)
                        elif event.button == 3:  # Right click - close gripper
                            new_pos = max(0.0, self.positions["Gripper"] - self.click_step)
                            self.set_joint("Gripper", new_pos)
                    elif event.type == pygame.MOUSEWHEEL:
                        # Scroll switches control mode
                        if event.y != 0:
                            self.switch_mode()
                
                # Mouse position control for active joints
                mouse_x, mouse_y = pygame.mouse.get_pos()
                
                # Calculate deltas from center
                delta_x = (mouse_x - center_x) * self.sensitivity
                delta_y = (center_y - mouse_y) * self.sensitivity
                
                # Apply to current joint pair
                joints = self.get_current_joints()
                self.set_joint(joints[0], delta_x)  # X controls first joint
                self.set_joint(joints[1], delta_y)  # Y controls second joint
                
                # Draw interface
                self.draw_interface()
                pygame.display.flip()
                clock.tick(30)
        finally:
            pygame.quit()
            self.stop_all()

if __name__ == "__main__":
    try:
        controller = Z1MouseControl()
        controller.run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        print("\nShutdown requested")
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)