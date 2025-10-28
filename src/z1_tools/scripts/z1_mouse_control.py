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
        pygame.display.set_caption("Z1 Safe Mouse Control - SIMULATION ONLY")
        
        # SAFE joint limits (conservative for simulation)
        self.limits = {
            0: (-1.0, 1.0),      # Joint 1: Base ±57° (safe)
            1: (0.2, 1.5),       # Joint 2: Shoulder 11° to 86° (avoid singularity)
            2: (-1.5, -0.2),     # Joint 3: Elbow -86° to -11° (safe range)
            3: (-1.0, 1.0),      # Joint 4: Forearm ±57° (safe)
            4: (-0.8, 0.8),      # Joint 5: Wrist pitch ±46° (safe)
            5: (-1.5, 1.5),      # Joint 6: Wrist roll ±86° (safe)
            6: (-1.57, 0.0)      # Gripper: -90° (closed) to 0° (open)
        }
        # Start in SAFE forward position (avoid singularities)
        self.positions = {0: 0.0, 1: 0.5, 2: -0.8, 3: 0.0, 4: 0.0, 5: 0.0, 6: -0.5}
        
        # SAFE control settings
        self.sensitivity = 0.001  # Reduced for safety
        self.click_step = 0.1     # Smaller steps
        self.running = True
        self.emergency_stop = False
        
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
        rospy.logwarn("🛑 EMERGENCY STOP - Moving to SAFE position")
        # Move to safe forward position (not zero - avoid singularities)
        safe_positions = {0: 0.0, 1: 0.5, 2: -0.8, 3: 0.0, 4: 0.0, 5: 0.0, 6: -1.57}
        for joint_id, safe_pos in safe_positions.items():
            self.set_joint(joint_id, safe_pos)
        print("Robot moved to SAFE forward position")
    
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
        help_y = 240
        help_texts = [
            "⚠️ SIMULATION ONLY - SAFE LIMITS",
            "Scroll: Switch control mode",
            "Left Click: Open gripper", 
            "Right Click: Close gripper",
            "Shift: Extra fine control (20%)",
            "ESC: EMERGENCY STOP"
        ]
        for text in help_texts:
            help_text = small_font.render(text, True, (180, 180, 180))
            self.screen.blit(help_text, (10, help_y))
            help_y += 18
    
    def run(self):
        print("Z1 SAFE Mouse Control Started - SIMULATION ONLY")
        print("⚠️  SAFETY WARNINGS:")
        print("   - This is for SIMULATION ONLY")
        print("   - Robot starts in SAFE forward position")
        print("   - Conservative joint limits enforced")
        print("   - ESC for immediate emergency stop")
        print("")
        print("Controls:")
        print("  Scroll Wheel: Switch control modes")
        print("  Mouse X/Y: Control active joint pair (SLOW)")
        print("  Left Click: Open gripper")
        print("  Right Click: Close gripper")
        print("  Shift: Fine control (30% speed)")
        print("  ESC: EMERGENCY STOP")
        
        # Move to safe starting position
        print("\nMoving to SAFE starting position...")
        for joint_id, pos in self.positions.items():
            self.set_joint(joint_id, pos)
        rospy.sleep(2.0)
        
        clock = pygame.time.Clock()
        center_x, center_y = 300, 200
        dead_zone = 50  # Dead zone radius where no movement occurs
        
        # Initialize mode
        self.switch_mode()
        
        try:
            while self.running and not rospy.is_shutdown():
                for event in pygame.event.get():
                    if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                        print("\n🛑 EMERGENCY STOP ACTIVATED")
                        self.emergency_stop = True
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
                
                # Only move if outside dead zone and not in emergency
                if distance > dead_zone and not self.emergency_stop:
                    # Scale movement based on distance beyond dead zone
                    scale = (distance - dead_zone) / (300 - dead_zone)  # Max distance ~300
                    delta_x = (dist_x / distance) * scale * self.sensitivity
                    delta_y = -(dist_y / distance) * scale * self.sensitivity
                    
                    # Modify sensitivity based on key modifiers
                    if self.shift_pressed:
                        delta_x *= 0.2  # Extra fine control
                        delta_y *= 0.2
                    # Remove fast control for safety
                    
                    # Apply to current joint pair with safety checks
                    joints = self.get_current_joints()
                    new_pos_x = self.positions[joints[0]] + delta_x
                    new_pos_y = self.positions[joints[1]] + delta_y
                    
                    # Extra safety: check if movement is within safe bounds
                    if (self.limits[joints[0]][0] <= new_pos_x <= self.limits[joints[0]][1] and
                        self.limits[joints[1]][0] <= new_pos_y <= self.limits[joints[1]][1]):
                        self.set_joint(joints[0], new_pos_x)
                        self.set_joint(joints[1], new_pos_y)
                
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