#!/usr/bin/env python3

"""
Z1 Mouse Control - Simple intuitive mouse control for Z1 robot
Follows Z1 SDK documentation and safety guidelines
"""

import rospy
import math
from unitree_legged_msgs.msg import MotorCmd

try:
    import pygame
except ImportError:
    print("Error: pygame not installed. Run: pip3 install pygame")
    exit(1)

class Z1MouseControl:
    def __init__(self):
        rospy.init_node("z1_mouse_control")
        
        pygame.init()
        self.screen = pygame.display.set_mode((400, 300))
        pygame.display.set_caption("Z1 Mouse Control")
        
        # Z1 joint limits (from SDK documentation)
        self.limits = {
            0: (-2.62, 2.62),    # Joint1: ±150°
            1: (0.0, 2.97),      # Joint2: 0° to 170°
            2: (-2.88, 0.0),     # Joint3: -165° to 0°
            3: (-1.52, 1.52),    # Joint4: ±87°
            4: (-1.34, 1.34),    # Joint5: ±77°
            5: (-2.79, 2.79),    # Joint6: ±160°
            6: (-1.57, 0.0)      # Gripper: -90° to 0°
        }
        
        self.positions = {i: 0.0 for i in range(7)}
        self.sensitivity = 0.003
        self.running = True
        self.mode = 0  # 0=Base/Shoulder, 1=Elbow/Forearm, 2=Wrist
        
        # Publishers
        self.pubs = {}
        for i in range(6):
            self.pubs[i] = rospy.Publisher(f"/z1_gazebo/Joint0{i+1}_controller/command", MotorCmd, queue_size=1)
        self.pubs[6] = rospy.Publisher("/z1_gazebo/Gripper_controller/command", MotorCmd, queue_size=1)

    def set_joint(self, joint_id, pos):
        min_pos, max_pos = self.limits[joint_id]
        pos = max(min_pos, min(max_pos, pos))
        self.positions[joint_id] = pos
        
        msg = MotorCmd()
        msg.mode = 10
        msg.q = float(pos)
        msg.dq = 0.0
        msg.tau = 0.0
        msg.Kp = 35.0
        msg.Kd = 1.5
        self.pubs[joint_id].publish(msg)
    
    def get_joints(self):
        modes = [[0, 1], [2, 3], [4, 5]]  # Base/Shoulder, Elbow/Forearm, Wrist
        return modes[self.mode]

    def draw_interface(self):
        self.screen.fill((30, 30, 30))
        font = pygame.font.Font(None, 20)
        
        mode_names = ["Base/Shoulder", "Elbow/Forearm", "Wrist"]
        joints = self.get_joints()
        
        # Mode display
        text = font.render(f"Mode: {mode_names[self.mode]} (J{joints[0]+1}/J{joints[1]+1})", True, (255, 255, 255))
        self.screen.blit(text, (10, 10))
        
        # Joint positions
        y = 40
        for i in range(7):
            name = f"J{i+1}" if i < 6 else "Grip"
            color = (0, 255, 0) if i in joints else (150, 150, 150)
            text = font.render(f"{name}: {self.positions[i]:.2f}", True, color)
            self.screen.blit(text, (10, y))
            y += 25
        
        # Controls
        controls = ["Scroll: Switch mode", "L/R Click: Gripper", "ESC: Stop"]
        y = 220
        for control in controls:
            text = font.render(control, True, (200, 200, 200))
            self.screen.blit(text, (10, y))
            y += 20

    def run(self):
        print("Z1 Mouse Control Started")
        print("Scroll: Switch modes | L/R Click: Gripper | ESC: Stop")
        
        clock = pygame.time.Clock()
        center_x, center_y = 200, 150
        
        try:
            while self.running and not rospy.is_shutdown():
                for event in pygame.event.get():
                    if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                        self.running = False
                    elif event.type == pygame.MOUSEBUTTONDOWN:
                        if event.button == 1:  # Open gripper
                            self.set_joint(6, min(0.0, self.positions[6] + 0.2))
                        elif event.button == 3:  # Close gripper
                            self.set_joint(6, max(-1.57, self.positions[6] - 0.2))
                    elif event.type == pygame.MOUSEWHEEL:
                        if event.y != 0:
                            self.mode = (self.mode + 1) % 3
                            mode_names = ["Base/Shoulder", "Elbow/Forearm", "Wrist"]
                            print(f"Mode: {mode_names[self.mode]}")
                
                # Mouse control
                mouse_x, mouse_y = pygame.mouse.get_pos()
                delta_x = (mouse_x - center_x) * self.sensitivity
                delta_y = (center_y - mouse_y) * self.sensitivity
                
                joints = self.get_joints()
                self.set_joint(joints[0], delta_x)
                self.set_joint(joints[1], delta_y)
                
                self.draw_interface()
                pygame.display.flip()
                clock.tick(30)
        finally:
            pygame.quit()
            for i in range(7):
                self.set_joint(i, 0.0)

if __name__ == "__main__":
    try:
        controller = Z1MouseControl()
        controller.run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        print("Shutdown")
    except Exception as e:
        print(f"Error: {e}")