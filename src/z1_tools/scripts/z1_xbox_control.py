#!/usr/bin/env python3
"""
Z1 Xbox Controller
Control the Z1 robotic arm with Xbox gamepad
Requires: sudo apt-get install ros-noetic-joy
"""
import rospy, math
from unitree_legged_msgs.msg import MotorCmd
from sensor_msgs.msg import Joy

class Z1XboxController:
    def __init__(self, mode=10, kp=35.0, kd=1.5):
        rospy.init_node("z1_xbox_control")
        
        self.mode = mode
        self.kp = kp
        self.kd = kd
        
        # Current joint positions
        self.positions = {
            "Joint01": 0.0,  # Base yaw
            "Joint02": 0.0,  # Shoulder pitch  
            "Joint03": 0.0,  # Elbow
            "Joint04": 0.0,  # Forearm roll
            "Joint05": 0.0,  # Wrist pitch
            "Joint06": 0.0,  # Wrist roll
            "Gripper": 0.0   # Gripper
        }
        
        # Safety limits
        self.limits = {
            "Joint01": (-1.2, 1.2),
            "Joint02": (-1.0, 1.0), 
            "Joint03": (0.0, 2.4),
            "Joint04": (-1.2, 1.2),
            "Joint05": (-1.0, 1.0),
            "Joint06": (-1.2, 1.2),
            "Gripper": (0.0, 0.8)
        }
        
        # Publishers
        self.pubs = {}
        for joint in self.positions.keys():
            topic = f"/z1_gazebo/{joint}_controller/command"
            self.pubs[joint] = rospy.Publisher(topic, MotorCmd, queue_size=10)
        
        # Joy subscriber
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)
        
        # Control parameters
        self.speed_multiplier = 1.0
        self.precision_mode = False
        self.last_joy = None
        
        self.rate = rospy.Rate(50)
        
        rospy.loginfo("🎮 Z1 Xbox Controller Ready!")
        rospy.loginfo("Controls:")
        rospy.loginfo("  Left Stick: Base rotation + Shoulder pitch")
        rospy.loginfo("  Right Stick: Elbow + Wrist pitch") 
        rospy.loginfo("  LT/RT: Forearm roll")
        rospy.loginfo("  LB/RB: Wrist roll")
        rospy.loginfo("  A/B: Gripper open/close")
        rospy.loginfo("  X: Precision mode toggle")
        rospy.loginfo("  Y: Emergency stop")
        rospy.loginfo("  Start: Speed up")
        rospy.loginfo("  Back: Speed down")
    
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
    
    def emergency_stop(self):
        """Return all joints to neutral"""
        rospy.logwarn("🛑 EMERGENCY STOP - Returning to neutral")
        for joint in self.positions.keys():
            self.positions[joint] = 0.0
            self.send_command(joint, 0.0)
    
    def joy_callback(self, msg):
        """Handle Xbox controller input"""
        if len(msg.axes) < 8 or len(msg.buttons) < 11:
            return
        
        # Xbox controller mapping (may vary by driver)
        # Axes: [LX, LY, LT, RX, RY, RT, DX, DY]
        # Buttons: [A, B, X, Y, LB, RB, Back, Start, Xbox, LS, RS]
        
        left_x = msg.axes[0]    # Left stick X (base rotation)
        left_y = msg.axes[1]    # Left stick Y (shoulder pitch)
        right_x = msg.axes[3]   # Right stick X (elbow)
        right_y = msg.axes[4]   # Right stick Y (wrist pitch)
        
        left_trigger = (1.0 - msg.axes[2]) / 2.0   # LT (0-1)
        right_trigger = (1.0 - msg.axes[5]) / 2.0  # RT (0-1)
        
        # Buttons
        btn_a = msg.buttons[0]      # A - Gripper close
        btn_b = msg.buttons[1]      # B - Gripper open
        btn_x = msg.buttons[2]      # X - Precision mode
        btn_y = msg.buttons[3]      # Y - Emergency stop
        btn_lb = msg.buttons[4]     # LB - Wrist roll left
        btn_rb = msg.buttons[5]     # RB - Wrist roll right
        btn_back = msg.buttons[6]   # Back - Speed down
        btn_start = msg.buttons[7]  # Start - Speed up
        
        # Handle button presses (only on press, not hold)
        if self.last_joy is not None:
            # Emergency stop
            if btn_y and not self.last_joy.buttons[3]:
                self.emergency_stop()
                return
            
            # Precision mode toggle
            if btn_x and not self.last_joy.buttons[2]:
                self.precision_mode = not self.precision_mode
                mode_str = "ON" if self.precision_mode else "OFF"
                rospy.loginfo(f"🎯 Precision mode: {mode_str}")
            
            # Speed control
            if btn_start and not self.last_joy.buttons[7]:
                self.speed_multiplier = min(2.0, self.speed_multiplier + 0.2)
                rospy.loginfo(f"⚡ Speed: {self.speed_multiplier:.1f}x")
            
            if btn_back and not self.last_joy.buttons[6]:
                self.speed_multiplier = max(0.2, self.speed_multiplier - 0.2)
                rospy.loginfo(f"🐌 Speed: {self.speed_multiplier:.1f}x")
        
        # Calculate movement deltas
        deadzone = 0.1
        speed = 0.02 * self.speed_multiplier
        
        if self.precision_mode:
            speed *= 0.3  # Slower in precision mode
        
        # Apply deadzone
        def apply_deadzone(value, zone=deadzone):
            return 0.0 if abs(value) < zone else value
        
        left_x = apply_deadzone(left_x)
        left_y = apply_deadzone(left_y)
        right_x = apply_deadzone(right_x)
        right_y = apply_deadzone(right_y)
        
        # Update joint positions based on controller input
        
        # Left stick - Base rotation and shoulder pitch
        if left_x != 0.0:
            delta = left_x * speed
            new_pos = self.positions["Joint01"] + delta
            self.positions["Joint01"] = self.clamp_position("Joint01", new_pos)
        
        if left_y != 0.0:
            delta = left_y * speed
            new_pos = self.positions["Joint02"] + delta
            self.positions["Joint02"] = self.clamp_position("Joint02", new_pos)
        
        # Right stick - Elbow and wrist pitch
        if right_x != 0.0:
            delta = right_x * speed
            new_pos = self.positions["Joint03"] + delta
            self.positions["Joint03"] = self.clamp_position("Joint03", new_pos)
        
        if right_y != 0.0:
            delta = right_y * speed
            new_pos = self.positions["Joint05"] + delta
            self.positions["Joint05"] = self.clamp_position("Joint05", new_pos)
        
        # Triggers - Forearm roll
        trigger_diff = right_trigger - left_trigger
        if abs(trigger_diff) > 0.1:
            delta = trigger_diff * speed
            new_pos = self.positions["Joint04"] + delta
            self.positions["Joint04"] = self.clamp_position("Joint04", new_pos)
        
        # Shoulder buttons - Wrist roll
        if btn_lb:
            delta = -speed
            new_pos = self.positions["Joint06"] + delta
            self.positions["Joint06"] = self.clamp_position("Joint06", new_pos)
        
        if btn_rb:
            delta = speed
            new_pos = self.positions["Joint06"] + delta
            self.positions["Joint06"] = self.clamp_position("Joint06", new_pos)
        
        # A/B buttons - Gripper
        if btn_a:  # Close gripper
            delta = -speed * 2
            new_pos = self.positions["Gripper"] + delta
            self.positions["Gripper"] = self.clamp_position("Gripper", new_pos)
        
        if btn_b:  # Open gripper
            delta = speed * 2
            new_pos = self.positions["Gripper"] + delta
            self.positions["Gripper"] = self.clamp_position("Gripper", new_pos)
        
        # Send commands to all joints
        for joint, position in self.positions.items():
            self.send_command(joint, position)
        
        self.last_joy = msg
    
    def run(self):
        """Main control loop"""
        rospy.loginfo("🎮 Xbox controller active - move sticks to control Z1!")
        rospy.loginfo("💡 Make sure to run: rosrun joy joy_node")
        
        while not rospy.is_shutdown():
            self.rate.sleep()

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Z1 Xbox Controller")
    parser.add_argument("--mode", type=int, default=10, help="Controller mode")
    parser.add_argument("--kp", type=float, default=35.0, help="Proportional gain")
    parser.add_argument("--kd", type=float, default=1.5, help="Derivative gain")
    args = parser.parse_args()
    
    try:
        controller = Z1XboxController(args.mode, args.kp, args.kd)
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Xbox controller stopped")

if __name__ == "__main__":
    main()