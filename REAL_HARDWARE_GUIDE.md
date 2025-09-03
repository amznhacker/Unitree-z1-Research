#!/usr/bin/env python3

"""
REAL HARDWARE SAFETY WRAPPER for Unitree Z1
⚠️  CRITICAL: Only use this script with real hardware
⚠️  NEVER use simulation scripts directly on real robot
"""

import rospy
import signal
import sys
import time
from unitree_legged_msgs.msg import MotorCmd, MotorState
from std_msgs.msg import Float32

class Z1RealHardwareSafe:
    def __init__(self):
        rospy.init_node('z1_real_hardware_safe', anonymous=True)
        
        # SAFETY PARAMETERS - DO NOT INCREASE WITHOUT TESTING
        self.SAFE_PARAMS = {
            'kp': 8.0,           # Conservative proportional gain
            'kd': 1.0,           # Conservative derivative gain  
            'max_velocity': 0.3,  # Maximum joint velocity (rad/s)
            'max_acceleration': 0.5,  # Maximum acceleration
            'position_tolerance': 0.02,  # Position accuracy
            'mode': 10           # Position control mode
        }
        
        # CONSERVATIVE JOINT LIMITS (80% of full range)
        self.SAFE_LIMITS = {
            'Joint01': (-1.0, 1.0),    # Base ±57°
            'Joint02': (-0.8, 0.8),    # Shoulder ±46°  
            'Joint03': (0.1, 2.0),     # Elbow 6° to 115°
            'Joint04': (-1.0, 1.0),    # Forearm ±57°
            'Joint05': (-0.8, 0.8),    # Wrist pitch ±46°
            'Joint06': (-1.0, 1.0),    # Wrist roll ±57°
        }
        
        self.joints = ['Joint01', 'Joint02', 'Joint03', 'Joint04', 'Joint05', 'Joint06']
        self.current_positions = {}
        self.target_positions = {}
        self.emergency_stop_active = False
        
        # Setup publishers and subscribers
        self.setup_communication()
        
        # Setup emergency stop
        signal.signal(signal.SIGINT, self.emergency_stop_handler)
        
        # Safety checks
        if not self.run_safety_checklist():
            rospy.logerr("SAFETY CHECKLIST FAILED - ABORTING")
            sys.exit(1)
            
        rospy.loginfo("Z1 Real Hardware Safe Controller Initialized")
        rospy.logwarn("EMERGENCY STOP: Press Ctrl+C or use hardware button")
        
    def setup_communication(self):
        """Setup ROS publishers and subscribers"""
        self.publishers = {}
        self.subscribers = {}
        
        for joint in self.joints:
            # Publishers for motor commands
            self.publishers[joint] = rospy.Publisher(
                f'/z1_real/{joint}_controller/command', 
                MotorCmd, queue_size=1
            )
            
            # Subscribers for motor states
            self.subscribers[joint] = rospy.Subscriber(
                f'/z1_real/{joint}_controller/state',
                MotorState, 
                lambda msg, j=joint: self.joint_state_callback(msg, j)
            )
            
        # Initialize positions
        for joint in self.joints:
            self.current_positions[joint] = 0.0
            self.target_positions[joint] = 0.0
            
    def joint_state_callback(self, msg, joint_name):
        """Update current joint positions"""
        self.current_positions[joint_name] = msg.q
        
    def run_safety_checklist(self):
        """Interactive safety checklist - MUST pass before operation"""
        print("\n" + "="*60)
        print("🚨 REAL HARDWARE SAFETY CHECKLIST 🚨")
        print("="*60)
        
        checks = [
            "Emergency stop button is accessible and tested",
            "Workspace is clear of people and obstacles", 
            "Robot is in safe starting position (joints near zero)",
            "All personnel are at safe distance (>2 meters)",
            "Monitoring camera/system is active",
            "Conservative parameters are loaded (Kp=8.0, max_vel=0.3)",
            "Joint limits are set to conservative ranges",
            "Communication with robot is stable",
            "You have read and understand the safety procedures",
            "You are prepared to hit emergency stop if needed"
        ]
        
        for i, check in enumerate(checks, 1):
            print(f"\n{i:2d}. {check}")
            while True:
                response = input("    Confirmed? (y/n/abort): ").lower().strip()
                if response == 'y':
                    print("    ✓ CONFIRMED")
                    break
                elif response == 'n':
                    print("    ✗ FAILED - Please address this issue")
                    return False
                elif response == 'abort':
                    print("    ⚠️  ABORTED BY USER")
                    return False
                else:
                    print("    Please enter 'y', 'n', or 'abort'")
                    
        print("\n✅ ALL SAFETY CHECKS PASSED")
        print("⚠️  Remember: Emergency stop is Ctrl+C or hardware button")
        return True
        
    def emergency_stop_handler(self, signum, frame):
        """Handle emergency stop signal"""
        print("\n🚨 EMERGENCY STOP ACTIVATED! 🚨")
        self.emergency_stop_active = True
        self.stop_all_motors()
        rospy.logwarn("EMERGENCY STOP: All motors disabled")
        sys.exit(0)
        
    def stop_all_motors(self):
        """Immediately stop all motors"""
        for joint in self.joints:
            cmd = MotorCmd()
            cmd.mode = 0  # Disable mode
            cmd.q = self.current_positions.get(joint, 0.0)
            cmd.dq = 0.0
            cmd.tau = 0.0
            cmd.Kp = 0.0
            cmd.Kd = 0.0
            self.publishers[joint].publish(cmd)
            
    def check_joint_limits(self, joint_name, target_position):
        """Check if target position is within safe limits"""
        if joint_name not in self.SAFE_LIMITS:
            rospy.logerr(f"Unknown joint: {joint_name}")
            return False
            
        min_pos, max_pos = self.SAFE_LIMITS[joint_name]
        if target_position < min_pos or target_position > max_pos:
            rospy.logerr(f"SAFETY VIOLATION: {joint_name} target {target_position:.3f} outside safe range [{min_pos:.3f}, {max_pos:.3f}]")
            return False
        return True
        
    def limit_velocity(self, joint_name, target_position):
        """Limit movement velocity for safety"""
        current_pos = self.current_positions.get(joint_name, 0.0)
        max_change = self.SAFE_PARAMS['max_velocity'] * 0.02  # 50Hz control loop
        
        position_diff = target_position - current_pos
        
        if abs(position_diff) > max_change:
            if position_diff > 0:
                limited_target = current_pos + max_change
            else:
                limited_target = current_pos - max_change
                
            rospy.logdebug(f"Velocity limited {joint_name}: {target_position:.3f} -> {limited_target:.3f}")
            return limited_target
            
        return target_position
        
    def move_joint_safe(self, joint_name, target_position):
        """Safely move a single joint with all safety checks"""
        if self.emergency_stop_active:
            return False
            
        # Safety check 1: Joint limits
        if not self.check_joint_limits(joint_name, target_position):
            return False
            
        # Safety check 2: Velocity limiting
        safe_target = self.limit_velocity(joint_name, target_position)
        
        # Safety check 3: Create safe command
        cmd = MotorCmd()
        cmd.mode = self.SAFE_PARAMS['mode']
        cmd.q = safe_target
        cmd.dq = 0.0
        cmd.tau = 0.0
        cmd.Kp = self.SAFE_PARAMS['kp']
        cmd.Kd = self.SAFE_PARAMS['kd']
        
        # Send command
        self.publishers[joint_name].publish(cmd)
        self.target_positions[joint_name] = safe_target
        
        return True
        
    def move_to_position(self, joint_positions, timeout=10.0):
        """Move multiple joints to target positions safely"""
        rospy.loginfo(f"Moving to position: {joint_positions}")
        
        start_time = time.time()
        
        while not rospy.is_shutdown() and not self.emergency_stop_active:
            all_reached = True
            
            for joint_name, target_pos in joint_positions.items():
                if joint_name in self.joints:
                    # Move joint safely
                    if not self.move_joint_safe(joint_name, target_pos):
                        rospy.logerr(f"Failed to move {joint_name} safely")
                        return False
                        
                    # Check if reached target
                    current_pos = self.current_positions.get(joint_name, 0.0)
                    if abs(current_pos - target_pos) > self.SAFE_PARAMS['position_tolerance']:
                        all_reached = False
                        
            if all_reached:
                rospy.loginfo("All joints reached target positions")
                return True
                
            if time.time() - start_time > timeout:
                rospy.logwarn("Movement timeout reached")
                return False
                
            rospy.sleep(0.02)  # 50Hz control loop
            
        return False
        
    def demo_safe_movement(self):
        """Demonstration of safe movement patterns"""
        rospy.loginfo("Starting safe movement demonstration...")
        
        # Home position
        home_position = {joint: 0.0 for joint in self.joints}
        
        # Safe test positions
        test_positions = [
            {'Joint01': 0.3, 'Joint02': 0.2, 'Joint03': 0.5},  # Small movements
            {'Joint01': -0.3, 'Joint02': -0.2, 'Joint03': 1.0},
            home_position  # Return home
        ]
        
        for i, position in enumerate(test_positions):
            rospy.loginfo(f"Moving to test position {i+1}")
            if not self.move_to_position(position):
                rospy.logerr("Movement failed, stopping demo")
                break
            rospy.sleep(2.0)  # Pause between movements
            
        rospy.loginfo("Safe movement demonstration completed")

if __name__ == '__main__':
    try:
        controller = Z1RealHardwareSafe()
        
        # Run demonstration
        controller.demo_safe_movement()
        
        # Keep node alive
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error: {e}")
        sys.exit(1)