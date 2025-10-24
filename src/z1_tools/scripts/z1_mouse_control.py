#!/usr/bin/env python3

"""
Z1 Mouse Control - Control robot with mouse/Mudra band
Mouse X/Y = Base/Shoulder, Left/Right Click = Gripper, Scroll = Elbow
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
        self.screen = pygame.display.set_mode((400, 300))
        pygame.display.set_caption("Z1 Mouse Control")
        
        # Joint limits and positions
        self.limits = {
            "Joint01": (-1.2, 1.2), "Joint02": (-1.0, 1.0), "Joint03": (0.0, 2.4),
            "Joint04": (-1.2, 1.2), "Joint05": (-1.0, 1.0), "Joint06": (-1.2, 1.2),
            "Gripper": (0.0, 0.6)
        }
        self.positions = {j: 0.0 for j in self.limits.keys()}
        
        # Control settings
        self.sensitivity = 0.002
        self.scroll_step = 0.1
        self.running = True
        
        # Publishers
        self.pubs = {}
        for joint in self.limits.keys():
            controller = f"{joint}_controller"
            topic = f"/z1_gazebo/{controller}/command"
            self.pubs[joint] = rospy.Publisher(topic, MotorCmd, queue_size=1)
        
        rospy.loginfo("Z1 Mouse Control initialized")
        
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
    
    def run(self):
        print("Z1 Mouse Control Started")
        print("Mouse X/Y=Base/Shoulder, Left/Right Click=Gripper, Scroll=Elbow, ESC=Quit")
        
        clock = pygame.time.Clock()
        center_x, center_y = 200, 150
        
        try:
            while self.running and not rospy.is_shutdown():
                for event in pygame.event.get():
                    if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                        self.running = False
                    elif event.type == pygame.MOUSEBUTTONDOWN:
                        if event.button == 1:  # Left click
                            self.set_joint("Gripper", min(0.6, self.positions["Gripper"] + 0.1))
                        elif event.button == 3:  # Right click
                            self.set_joint("Gripper", max(0.0, self.positions["Gripper"] - 0.1))
                    elif event.type == pygame.MOUSEWHEEL:
                        delta = event.y * self.scroll_step
                        self.set_joint("Joint03", self.positions["Joint03"] + delta)
                
                # Mouse position control
                mouse_x, mouse_y = pygame.mouse.get_pos()
                self.set_joint("Joint01", (mouse_x - center_x) * self.sensitivity)
                self.set_joint("Joint02", (center_y - mouse_y) * self.sensitivity)
                
                self.screen.fill((0, 0, 0))
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