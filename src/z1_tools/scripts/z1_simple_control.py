#!/usr/bin/env python3

"""
Simple Z1 Control - Works with Unitree MotorCmd messages
Controls: WASD=base/shoulder, ZE=elbow, RF=forearm, TG=wrist_pitch, YH=wrist_roll
Space=open gripper, X=close gripper, ESC=stop
"""

import rospy
import sys
import termios
import tty
import threading
from unitree_legged_msgs.msg import MotorCmd

class SimpleZ1Control:
    def __init__(self):
        rospy.init_node("z1_simple_control")
        
        # Joint limits (conservative)
        self.limits = {
            "Joint01": (-1.2, 1.2),   # Base ±69°
            "Joint02": (-1.0, 1.0),   # Shoulder ±57°
            "Joint03": (0.0, 2.4),    # Elbow 0° to 137°
            "Joint04": (-1.2, 1.2),   # Forearm ±69°
            "Joint05": (-1.0, 1.0),   # Wrist pitch ±57°
            "Joint06": (-1.2, 1.2),   # Wrist roll ±69°
            "Gripper": (0.0, 0.6)     # Gripper 0-60%
        }
        
        # Current positions
        self.positions = {j: 0.0 for j in self.limits.keys()}
        
        # Control settings
        self.step_size = 0.05
        self.running = True
        
        # Publishers for Unitree controllers
        self.pubs = {}
        controller_map = {
            "Joint01": "Joint01_controller",
            "Joint02": "Joint02_controller", 
            "Joint03": "Joint03_controller",
            "Joint04": "Joint04_controller",
            "Joint05": "Joint05_controller",
            "Joint06": "Joint06_controller",
            "Gripper": "Gripper_controller"
        }
        
        for joint, controller in controller_map.items():
            topic = f"/z1_gazebo/{controller}/command"
            self.pubs[joint] = rospy.Publisher(topic, MotorCmd, queue_size=1)
        
        rospy.loginfo("Z1 Simple Control initialized with Unitree controllers")
        
    def clamp(self, joint, pos):
        """Keep position within safe limits"""
        min_pos, max_pos = self.limits[joint]
        return max(min_pos, min(max_pos, pos))
    
    def move_joint(self, joint, delta):
        """Move joint by delta amount"""
        new_pos = self.positions[joint] + delta
        self.positions[joint] = self.clamp(joint, new_pos)
        
        # Create MotorCmd message
        msg = MotorCmd()
        msg.mode = 10  # Position control mode
        msg.q = float(self.positions[joint])
        msg.dq = 0.0
        msg.tau = 0.0
        msg.Kp = 35.0
        msg.Kd = 1.5
        
        self.pubs[joint].publish(msg)
    
    def stop_all(self):
        """Emergency stop - return to neutral"""
        rospy.logwarn("STOPPING - Returning to neutral position")
        for joint in self.positions.keys():
            self.positions[joint] = 0.0
            msg = MotorCmd()
            msg.mode = 10
            msg.q = 0.0
            msg.dq = 0.0
            msg.tau = 0.0
            msg.Kp = 35.0
            msg.Kd = 1.5
            self.pubs[joint].publish(msg)
    
    def keyboard_input(self):
        """Handle keyboard input"""
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            
            while self.running and not rospy.is_shutdown():
                char = sys.stdin.read(1)
                
                if char == '\x1b':  # ESC
                    self.stop_all()
                    self.running = False
                    break
                elif char == 'q':
                    self.running = False
                    break
                
                # Movement controls
                elif char == 'w':  # Shoulder up
                    self.move_joint("Joint02", self.step_size)
                elif char == 's':  # Shoulder down
                    self.move_joint("Joint02", -self.step_size)
                elif char == 'a':  # Base left
                    self.move_joint("Joint01", self.step_size)
                elif char == 'd':  # Base right
                    self.move_joint("Joint01", -self.step_size)
                
                elif char == 'z':  # Elbow bend
                    self.move_joint("Joint03", self.step_size)
                elif char == 'e':  # Elbow extend
                    self.move_joint("Joint03", -self.step_size)
                
                elif char == 'r':  # Forearm roll left
                    self.move_joint("Joint04", self.step_size)
                elif char == 'f':  # Forearm roll right
                    self.move_joint("Joint04", -self.step_size)
                
                elif char == 't':  # Wrist pitch up
                    self.move_joint("Joint05", self.step_size)
                elif char == 'g':  # Wrist pitch down
                    self.move_joint("Joint05", -self.step_size)
                
                elif char == 'y':  # Wrist roll left
                    self.move_joint("Joint06", self.step_size)
                elif char == 'h':  # Wrist roll right
                    self.move_joint("Joint06", -self.step_size)
                
                elif char == ' ':  # Open gripper
                    self.move_joint("Gripper", self.step_size)
                elif char == 'x':  # Close gripper
                    self.move_joint("Gripper", -self.step_size)
                
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
    def run(self):
        """Main control loop"""
        print("Z1 Simple Control Started")
        print("Controls:")
        print("  WASD = Base/Shoulder movement")
        print("  ZE   = Elbow bend/extend")
        print("  RF   = Forearm roll")
        print("  TG   = Wrist pitch")
        print("  YH   = Wrist roll")
        print("  Space = Open gripper")
        print("  X     = Close gripper")
        print("  ESC   = Emergency stop")
        print("  Q     = Quit")
        
        # Start keyboard thread
        kb_thread = threading.Thread(target=self.keyboard_input)
        kb_thread.daemon = True
        kb_thread.start()
        
        # Status display
        rate = rospy.Rate(10)
        while self.running and not rospy.is_shutdown():
            # Print current positions
            status = "Positions: "
            for joint, pos in self.positions.items():
                if joint == "Gripper":
                    status += f"{joint}:{pos:.2f} "
                else:
                    status += f"{joint}:{pos:.2f}° "
            print(f"\r{status}", end="", flush=True)
            
            rate.sleep()
        
        print("\nShutting down...")
        self.stop_all()

if __name__ == "__main__":
    try:
        controller = SimpleZ1Control()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nShutdown requested")
    except Exception as e:
        print(f"Error: {e}")