#!/usr/bin/env python3
"""
Z1 Keyboard Control Script
Based on: https://dev-z1.unitree.com/use/keyboard.html

Controls:
- WASD: Base rotation and shoulder pitch
- QE: Elbow bend
- RF: Forearm roll  
- TG: Wrist pitch
- YH: Wrist roll
- Space: Open gripper
- X: Close gripper
- ESC: Emergency stop (return to neutral)
- Numbers 1-6: Select joint for fine control
- Arrow keys: Fine joint control when joint selected
"""
import rospy, sys, termios, tty, threading, time
from unitree_legged_msgs.msg import MotorCmd

class Z1KeyboardControl:
    def __init__(self):
        rospy.init_node("z1_keyboard_control")
        
        # Safety limits (radians)
        self.limits = {
            "Joint01": (-1.57, 1.57),   # ±90°
            "Joint02": (-1.22, 1.22),   # ±70°
            "Joint03": (-0.17, 2.61),   # -10° to 150°
            "Joint04": (-1.57, 1.57),   # ±90°
            "Joint05": (-1.22, 1.22),   # ±70°
            "Joint06": (-1.57, 1.57),   # ±90°
            "Gripper": (0.0, 0.8)       # 0 to 80% open
        }
        
        # Current positions
        self.positions = {j: 0.0 for j in self.limits.keys()}
        
        # Control parameters
        self.mode = 10
        self.kp = 35.0
        self.kd = 1.5
        self.step_size = 0.05  # radians per keypress
        self.selected_joint = None
        
        # Publishers
        self.pubs = {}
        for joint in self.limits.keys():
            topic = f"/z1_gazebo/{joint}_controller/command"
            self.pubs[joint] = rospy.Publisher(topic, MotorCmd, queue_size=10)
        
        self.running = True
        self.rate = rospy.Rate(50)
        
    def clamp_position(self, joint, pos):
        """Clamp position to safe limits"""
        min_pos, max_pos = self.limits[joint]
        return max(min_pos, min(max_pos, pos))
    
    def send_command(self, joint, position):
        """Send motor command"""
        msg = MotorCmd()
        msg.mode = self.mode
        msg.q = float(position)
        msg.dq = 0.0
        msg.tau = 0.0
        msg.Kp = self.kp
        msg.Kd = self.kd
        self.pubs[joint].publish(msg)
    
    def move_joint(self, joint, delta):
        """Move joint by delta amount"""
        new_pos = self.positions[joint] + delta
        self.positions[joint] = self.clamp_position(joint, new_pos)
        self.send_command(joint, self.positions[joint])
    
    def emergency_stop(self):
        """Return all joints to neutral position"""
        rospy.logwarn("EMERGENCY STOP - Returning to neutral")
        for joint in self.positions.keys():
            self.positions[joint] = 0.0
            self.send_command(joint, 0.0)
    
    def print_status(self):
        """Print current status"""
        print("\\r\\033[K", end="")  # Clear line
        status = f"Selected: {self.selected_joint or 'None'} | "
        for joint, pos in self.positions.items():
            if joint == "Gripper":
                status += f"{joint}: {pos:.2f} | "
            else:
                status += f"{joint}: {pos:.2f}° | "
        print(status, end="", flush=True)
    
    def keyboard_thread(self):
        """Handle keyboard input"""
        # Set terminal to raw mode
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            
            while self.running and not rospy.is_shutdown():
                char = sys.stdin.read(1)
                
                if char == '\\x1b':  # ESC
                    self.emergency_stop()
                elif char == 'q':
                    self.running = False
                    break
                
                # Joint selection (1-6)
                elif char in '123456':
                    joint_num = int(char)
                    self.selected_joint = f"Joint0{joint_num}"
                    rospy.loginfo(f"Selected {self.selected_joint}")
                
                # Main controls
                elif char == 'w':  # Shoulder pitch up
                    self.move_joint("Joint02", self.step_size)
                elif char == 's':  # Shoulder pitch down
                    self.move_joint("Joint02", -self.step_size)
                elif char == 'a':  # Base rotate left
                    self.move_joint("Joint01", self.step_size)
                elif char == 'd':  # Base rotate right
                    self.move_joint("Joint01", -self.step_size)
                
                elif char == 'q':  # Elbow bend
                    self.move_joint("Joint03", self.step_size)
                elif char == 'e':  # Elbow extend
                    self.move_joint("Joint03", -self.step_size)
                
                elif char == 'r':  # Forearm roll CCW
                    self.move_joint("Joint04", self.step_size)
                elif char == 'f':  # Forearm roll CW
                    self.move_joint("Joint04", -self.step_size)
                
                elif char == 't':  # Wrist pitch up
                    self.move_joint("Joint05", self.step_size)
                elif char == 'g':  # Wrist pitch down
                    self.move_joint("Joint05", -self.step_size)
                
                elif char == 'y':  # Wrist roll CCW
                    self.move_joint("Joint06", self.step_size)
                elif char == 'h':  # Wrist roll CW
                    self.move_joint("Joint06", -self.step_size)
                
                elif char == ' ':  # Open gripper
                    self.move_joint("Gripper", self.step_size)
                elif char == 'x':  # Close gripper
                    self.move_joint("Gripper", -self.step_size)
                
                # Fine control with arrow keys
                elif char == '\\x1b':  # Arrow key sequence
                    char2 = sys.stdin.read(1)
                    if char2 == '[':
                        char3 = sys.stdin.read(1)
                        if self.selected_joint:
                            if char3 == 'A':  # Up arrow
                                self.move_joint(self.selected_joint, self.step_size/5)
                            elif char3 == 'B':  # Down arrow
                                self.move_joint(self.selected_joint, -self.step_size/5)
                
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
    def run(self):
        """Main control loop"""
        print("Z1 Keyboard Control Started")
        print("Controls: WASD=base/shoulder, QE=elbow, RF=forearm, TG=wrist_pitch, YH=wrist_roll")
        print("Space=open gripper, X=close gripper, ESC=emergency stop, Q=quit")
        print("Numbers 1-6=select joint, Arrow keys=fine control")
        
        # Start keyboard thread
        kb_thread = threading.Thread(target=self.keyboard_thread)
        kb_thread.daemon = True
        kb_thread.start()
        
        # Main loop - send commands and update display
        while self.running and not rospy.is_shutdown():
            self.print_status()
            self.rate.sleep()
        
        print("\\nShutting down...")
        self.emergency_stop()

if __name__ == "__main__":
    try:
        controller = Z1KeyboardControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass