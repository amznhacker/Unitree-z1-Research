#!/usr/bin/env python3

"""
Z1 Xbox Controller - Control Unitree Z1 with Xbox gamepad
Requires: sudo apt install ros-noetic-joy
Connect Xbox controller via USB or Bluetooth before running
"""

import rospy
import math
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

class Z1XboxControl:
    def __init__(self):
        rospy.init_node("z1_xbox_control")
        
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
        self.speed_multiplier = 0.02  # Adjust for sensitivity
        
        # Publishers
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
            self.pubs[joint] = rospy.Publisher(topic, Float64, queue_size=1)
        
        # Subscribe to joy messages
        rospy.Subscriber("/joy", Joy, self.joy_callback)
        
        rospy.loginfo("🎮 Z1 Xbox Controller initialized")
        rospy.loginfo("📋 Xbox Controls:")
        rospy.loginfo("  Left Stick: Base rotation & Shoulder pitch")
        rospy.loginfo("  Right Stick: Elbow & Forearm roll")
        rospy.loginfo("  D-Pad: Wrist pitch & roll")
        rospy.loginfo("  RT/LT: Gripper open/close")
        rospy.loginfo("  Back: Emergency stop (return to home)")
        
    def clamp(self, joint, pos):
        """Keep position within safe limits"""
        min_pos, max_pos = self.limits[joint]
        return max(min_pos, min(max_pos, pos))
    
    def move_joint(self, joint, delta):
        """Move joint by delta amount"""
        new_pos = self.positions[joint] + delta
        self.positions[joint] = self.clamp(joint, new_pos)
        
        # Create Float64 message for Gazebo controllers
        msg = Float64()
        msg.data = float(self.positions[joint])
        
        self.pubs[joint].publish(msg)
    
    def emergency_stop(self):
        """Return all joints to home position"""
        rospy.logwarn("🛑 Emergency stop - returning to home position")
        for joint in self.positions.keys():
            self.positions[joint] = 0.0
            msg = Float64()
            msg.data = 0.0
            self.pubs[joint].publish(msg)
    
    def joy_callback(self, msg):
        """Handle Xbox controller input"""
        if len(msg.axes) < 8 or len(msg.buttons) < 11:
            return  # Not enough inputs
        
        # Xbox controller mapping (standard)
        # Axes: [LX, LY, LT, RX, RY, RT, DX, DY]
        # Buttons: [A, B, X, Y, LB, RB, Back, Start, Xbox, LS, RS]
        
        left_x = msg.axes[0]    # Left stick X (base rotation)
        left_y = msg.axes[1]    # Left stick Y (shoulder pitch)
        right_x = msg.axes[3]   # Right stick X (forearm roll)
        right_y = msg.axes[4]   # Right stick Y (elbow)
        dpad_x = msg.axes[6]    # D-pad X (wrist roll)
        dpad_y = msg.axes[7]    # D-pad Y (wrist pitch)
        
        left_trigger = msg.axes[2]   # LT (close gripper)
        right_trigger = msg.axes[5]  # RT (open gripper)
        
        back_button = msg.buttons[6]  # Back button (emergency stop)
        
        # Apply deadzone
        deadzone = 0.1
        def apply_deadzone(value):
            return value if abs(value) > deadzone else 0.0
        
        left_x = apply_deadzone(left_x)
        left_y = apply_deadzone(left_y)
        right_x = apply_deadzone(right_x)
        right_y = apply_deadzone(right_y)
        dpad_x = apply_deadzone(dpad_x)
        dpad_y = apply_deadzone(dpad_y)
        
        # Emergency stop
        if back_button:
            self.emergency_stop()
            return
        
        # Joint movements
        if left_x != 0:  # Base rotation
            self.move_joint("Joint01", -left_x * self.speed_multiplier)
        
        if left_y != 0:  # Shoulder pitch
            self.move_joint("Joint02", left_y * self.speed_multiplier)
        
        if right_y != 0:  # Elbow
            self.move_joint("Joint03", -right_y * self.speed_multiplier)
        
        if right_x != 0:  # Forearm roll
            self.move_joint("Joint04", right_x * self.speed_multiplier)
        
        if dpad_y != 0:  # Wrist pitch
            self.move_joint("Joint05", dpad_y * self.speed_multiplier)
        
        if dpad_x != 0:  # Wrist roll
            self.move_joint("Joint06", dpad_x * self.speed_multiplier)
        
        # Gripper control (triggers)
        # Triggers range from 1 (not pressed) to -1 (fully pressed)
        if right_trigger < 0.9:  # RT pressed (open gripper)
            grip_delta = (0.9 - right_trigger) * self.speed_multiplier
            self.move_joint("Gripper", grip_delta)
        
        if left_trigger < 0.9:  # LT pressed (close gripper)
            grip_delta = -(0.9 - left_trigger) * self.speed_multiplier
            self.move_joint("Gripper", grip_delta)
    
    def run(self):
        """Main control loop"""
        rospy.loginfo("🎮 Xbox controller ready! Connect your Xbox controller and start playing.")
        rospy.loginfo("💡 Tip: If controller not detected, run: ls /dev/input/js*")
        
        rate = rospy.Rate(50)  # 50Hz
        while not rospy.is_shutdown():
            # Print current positions occasionally
            if rospy.get_time() % 2 < 0.02:  # Every 2 seconds
                status = "Positions: "
                for joint, pos in self.positions.items():
                    if joint == "Gripper":
                        status += f"{joint}:{pos:.2f} "
                    else:
                        status += f"{joint}:{pos:.2f}° "
                rospy.loginfo_throttle(2, status)
            
            rate.sleep()

if __name__ == "__main__":
    try:
        controller = Z1XboxControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("🛑 Xbox controller stopped")
    except Exception as e:
        rospy.logerr(f"❌ Xbox controller error: {e}")
        rospy.logerr("💡 Make sure Xbox controller is connected and joy_node is running")