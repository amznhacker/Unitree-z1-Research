#!/usr/bin/env python3

"""
Z1 Enhanced Mouse Control - Full 6-DOF + gripper control using proper Z1 SDK interface
"""

import rospy
import sys
import math
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
        pygame.display.set_caption("Z1 Full Mouse Control")
        
        # Joint limits (from Z1 SDK documentation)
        self.limits = {
            0: (-2.62, 2.62),    # Joint 1: Base ±150°
            1: (0.0, 2.97),      # Joint 2: Shoulder 0° to 170°
            2: (-2.88, 0.0),     # Joint 3: Elbow -165° to 0°
            3: (-1.52, 1.52),    # Joint 4: Forearm ±87°
            4: (-1.34, 1.34),    # Joint 5: Wrist pitch ±77°
            5: (-2.79, 2.79),    # Joint 6: Wrist roll ±160°
            6: (-1.57, 0.0)      # Gripper: -90° (closed) to 0° (open)
        }
        self.positions = {i: 0.0 for i in range(7)}
        
        # Control settings
        self.sensitivity = 0.002
        self.click_step = 0.2
        self.running = True
        
        # Control modes - joint pairs
        self.modes = [
            [0, 1],  # Base & Shoulder
            [2, 3],  # Elbow & Forearm
            [4, 5]   # Wrist pitch & roll
        ]
        self.mode_index = 0
        
        # Key states for modifiers
        self.shift_pressed = False
        self.ctrl_pressed = False
        
        # Publishers for Z1 joints
        self.pubs = {}
        for i in range(6):  # 6 arm joints
            topic = f"/z1_gazebo/Joint0{i+1}_controller/command"
            self.pubs[i] = rospy.Publisher(topic, MotorCmd, queue_size=1)
        
        # Gripper publisher
        self.pubs[6] = rospy.Publisher("/z1_gazebo/Gripper_controller/command", MotorCmd, queue_size=1)
        
        rospy.loginfo("Z1 Enhanced Mouse Control initialized")
        
    def clamp(self, joint_id, pos):
        min_pos, max_pos = self.limits[joint_id]
        return max(min_pos, min(max_pos, pos))
    
    def set_joint(self, joint_id, pos):
        self.positions[joint_id] = self.clamp(joint_id, pos)
        
        msg = MotorCmd()
        msg.mode = 10  # Position control mode
        msg.q = float(self.positions[joint_id])
        msg.dq = 0.0
        msg.tau = 0.0
        msg.Kp = 35.0
        msg.Kd = 1.5
        
        self.pubs[joint_id].publish(msg)
    
    def stop_all(self):
        rospy.logwarn("EMERGENCY STOP - Returning to neutral")
        for joint_id in range(6):  # Arm joints to neutral
            self.set_joint(joint_id, 0.0)
        self.set_joint(6, -1.57)  # Close gripper for safety
    
    def get_current_joints(self):
        """Get current control joints based on mode"""
        return self.modes[self.mode_index]
    
    def switch_mode(self):
        """Switch to next control mode"""
        self.mode_index = (self.mode_index + 1) % len(self.modes)
        joints = self.modes[self.mode_index]
        mode_names = ["Base & Shoulder", "Elbow & Forearm", "Wrist Pitch & Roll"]
        print(f"\nMode: {mode_names[self.mode_index]} - Controlling Joint{joints[0]} (X) & Joint{joints[1]} (Y)")
    
    def draw_interface(self):
        """Draw control interface"""
        self.screen.fill((20, 20, 30))
        
        # Create font
        font = pygame.font.Font(None, 24)
        small_font = pygame.font.Font(None, 18)
        
        # Current mode
        mode_names = ["Base & Shoulder", "Elbow & Forearm", "Wrist Pitch & Roll"]
        joints = self.get_current_joints()
        mode_text = font.render(f"Mode: {mode_names[self.mode_index]}", True, (255, 255, 255))
        self.screen.blit(mode_text, (10, 10))
        
        control_text = small_font.render(f"Mouse X/Y controls Joint{joints[0]} & Joint{joints[1]}", True, (200, 200, 200))
        self.screen.blit(control_text, (10, 35))
        
        # Joint positions
        y_pos = 70
        for joint_id, pos in self.positions.items():
            color = (0, 255, 0) if joint_id in joints else (150, 150, 150)
            if joint_id < 6:
                joint_name = f"Joint{joint_id}"
                text = small_font.render(f"{joint_name}: {pos:.3f}rad", True, color)
            else:
                status = "OPEN" if pos > -0.3 else "CLOSED" if pos < -1.2 else "NEUTRAL"
                text = small_font.render(f"Gripper: {pos:.3f} ({status})", True, color)
            self.screen.blit(text, (10, y_pos))
            y_pos += 20
        
        # Controls help
        help_y = 260
        help_texts = [
            "Scroll: Switch control mode",
            "Left Click: Open gripper", 
            "Right Click: Close gripper",
            "Shift: Fine control (30%)",
            "Ctrl: Fast control (300%)",
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
        print("  Shift: Fine control (30% speed)")
        print("  Ctrl: Fast control (300% speed)")
        print("  ESC: Emergency stop")
        
        clock = pygame.time.Clock()
        center_x, center_y = 300, 200
        dead_zone = 50  # Dead zone radius where no movement occurs
        
        # Initialize mode
        self.switch_mode()
        
        try:
            while self.running and not rospy.is_shutdown():
                for event in pygame.event.get():
                    if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                        self.running = False
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_LSHIFT or event.key == pygame.K_RSHIFT:
                            self.shift_pressed = True
                        elif event.key == pygame.K_LCTRL or event.key == pygame.K_RCTRL:
                            self.ctrl_pressed = True
                    elif event.type == pygame.KEYUP:
                        if event.key == pygame.K_LSHIFT or event.key == pygame.K_RSHIFT:
                            self.shift_pressed = False
                        elif event.key == pygame.K_LCTRL or event.key == pygame.K_RCTRL:
                            self.ctrl_pressed = False
                    elif event.type == pygame.MOUSEBUTTONDOWN:
                        if event.button == 1:  # Left click - open gripper
                            new_pos = min(0.0, self.positions[6] + self.click_step)
                            self.set_joint(6, new_pos)
                            print(f"Opening gripper to {new_pos:.2f}")
                        elif event.button == 3:  # Right click - close gripper
                            new_pos = max(-1.57, self.positions[6] - self.click_step)
                            self.set_joint(6, new_pos)
                            print(f"Closing gripper to {new_pos:.2f}")
                    elif event.type == pygame.MOUSEWHEEL:
                        # Scroll switches control mode
                        if event.y != 0:
                            self.switch_mode()
                
                # Mouse position control with dead zone
                mouse_x, mouse_y = pygame.mouse.get_pos()
                
                # Calculate distance from center
                dist_x = mouse_x - center_x
                dist_y = mouse_y - center_y
                distance = math.sqrt(dist_x*dist_x + dist_y*dist_y)
                
                # Only move if outside dead zone
                if distance > dead_zone:
                    # Scale movement based on distance beyond dead zone
                    scale = (distance - dead_zone) / (300 - dead_zone)  # Max distance ~300
                    delta_x = (dist_x / distance) * scale * self.sensitivity * 2
                    delta_y = -(dist_y / distance) * scale * self.sensitivity * 2
                    
                    # Modify sensitivity based on key modifiers
                    if self.shift_pressed:
                        delta_x *= 0.3  # Fine control
                        delta_y *= 0.3
                    elif self.ctrl_pressed:
                        delta_x *= 3.0  # Fast control
                        delta_y *= 3.0
                    
                    # Apply to current joint pair
                    joints = self.get_current_joints()
                    self.set_joint(joints[0], delta_x)
                    self.set_joint(joints[1], delta_y)
                
                # Draw interface with dead zone indicator
                self.draw_interface()
                # Draw dead zone circle
                pygame.draw.circle(self.screen, (100, 100, 100), (center_x, center_y), dead_zone, 2)
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