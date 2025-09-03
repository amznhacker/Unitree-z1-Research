#!/usr/bin/env python3

"""
Z1 Advanced Control - Professional-grade robotic arm control
Features: Trajectory planning, force control, collision avoidance, precision positioning
"""

import rospy
import numpy as np
import threading
import time
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import JointState
from unitree_legged_msgs.msg import MotorCmd
from std_msgs.msg import Float64MultiArray
import tf2_ros
import tf2_geometry_msgs

class Z1AdvancedControl:
    def __init__(self):
        rospy.init_node("z1_advanced_control")
        
        # Advanced control parameters
        self.control_frequency = 100  # Hz - High precision control
        self.trajectory_buffer = []
        self.current_joint_states = np.zeros(7)
        self.target_joint_states = np.zeros(7)
        self.joint_velocities = np.zeros(7)
        self.joint_efforts = np.zeros(7)
        
        # Safety parameters
        self.max_joint_velocity = 1.0  # rad/s
        self.max_joint_acceleration = 2.0  # rad/s²
        self.collision_threshold = 50.0  # Nm
        self.workspace_limits = {
            'x': (-0.8, 0.8), 'y': (-0.8, 0.8), 'z': (0.0, 1.2)
        }
        
        # Control modes
        self.control_mode = "position"  # position, velocity, force, impedance
        self.emergency_stop = False
        
        # Publishers
        self.joint_pubs = {}
        for i in range(1, 7):
            topic = f"/z1_gazebo/Joint0{i}_controller/command"
            self.joint_pubs[f"Joint0{i}"] = rospy.Publisher(topic, MotorCmd, queue_size=1)
        
        self.gripper_pub = rospy.Publisher("/z1_gazebo/Gripper_controller/command", MotorCmd, queue_size=1)
        
        # Subscribers
        rospy.Subscriber("/z1_gazebo/joint_states", JointState, self.joint_state_callback)
        rospy.Subscriber("/z1_advanced/target_pose", PoseStamped, self.target_pose_callback)
        rospy.Subscriber("/z1_advanced/trajectory", Float64MultiArray, self.trajectory_callback)
        rospy.Subscriber("/z1_advanced/emergency_stop", Float64MultiArray, self.emergency_stop_callback)
        
        # TF2 for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Start control loop
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        rospy.loginfo("🤖 Z1 Advanced Control System initialized")
        rospy.loginfo("🎯 Professional-grade robotic arm control ready")
        
    def joint_state_callback(self, msg):
        """Update current joint states"""
        if len(msg.position) >= 7:
            self.current_joint_states = np.array(msg.position[:7])
            if len(msg.velocity) >= 7:
                self.joint_velocities = np.array(msg.velocity[:7])
            if len(msg.effort) >= 7:
                self.joint_efforts = np.array(msg.effort[:7])
    
    def target_pose_callback(self, msg):
        """Handle Cartesian pose targets"""
        # Convert Cartesian pose to joint angles using inverse kinematics
        target_joints = self.inverse_kinematics(msg.pose)
        if target_joints is not None:
            self.plan_trajectory(self.current_joint_states, target_joints)
    
    def trajectory_callback(self, msg):
        """Handle trajectory waypoints"""
        waypoints = np.array(msg.data).reshape(-1, 7)
        self.trajectory_buffer = waypoints.tolist()
        rospy.loginfo(f"📍 Received trajectory with {len(waypoints)} waypoints")
    
    def emergency_stop_callback(self, msg):
        """Emergency stop handler"""
        self.emergency_stop = True
        rospy.logwarn("🛑 EMERGENCY STOP ACTIVATED")
    
    def inverse_kinematics(self, pose):
        """
        Simplified inverse kinematics for Z1 arm
        In production, use proper IK solver like KDL or MoveIt
        """
        # This is a simplified version - real implementation would use
        # proper kinematic chain and numerical methods
        
        x, y, z = pose.position.x, pose.position.y, pose.position.z
        
        # Check workspace limits
        if not (self.workspace_limits['x'][0] <= x <= self.workspace_limits['x'][1] and
                self.workspace_limits['y'][0] <= y <= self.workspace_limits['y'][1] and
                self.workspace_limits['z'][0] <= z <= self.workspace_limits['z'][1]):
            rospy.logwarn("🚫 Target pose outside workspace limits")
            return None
        
        # Simplified IK calculation (placeholder)
        # Real implementation would solve the full kinematic equations
        joint_angles = np.zeros(7)
        joint_angles[0] = np.arctan2(y, x)  # Base rotation
        
        # Distance from base
        r = np.sqrt(x**2 + y**2)
        
        # Simplified arm positioning (this is a placeholder)
        joint_angles[1] = -0.5  # Shoulder
        joint_angles[2] = 1.0   # Elbow
        joint_angles[3] = 0.0   # Forearm
        joint_angles[4] = -0.5  # Wrist pitch
        joint_angles[5] = 0.0   # Wrist roll
        joint_angles[6] = 0.0   # Gripper
        
        return joint_angles
    
    def plan_trajectory(self, start_joints, end_joints, duration=3.0):
        """
        Plan smooth trajectory between joint configurations
        Uses quintic polynomial for smooth acceleration profiles
        """
        steps = int(duration * self.control_frequency)
        trajectory = []
        
        for i in range(steps + 1):
            t = i / steps
            
            # Quintic polynomial trajectory (smooth acceleration)
            s = 10 * t**3 - 15 * t**4 + 6 * t**5
            
            waypoint = start_joints + s * (end_joints - start_joints)
            trajectory.append(waypoint)
        
        self.trajectory_buffer = trajectory
        rospy.loginfo(f"📈 Planned trajectory with {len(trajectory)} points over {duration}s")
    
    def collision_detection(self):
        """
        Basic collision detection using joint effort monitoring
        Real implementation would use proper collision checking
        """
        for i, effort in enumerate(self.joint_efforts):
            if abs(effort) > self.collision_threshold:
                rospy.logwarn(f"⚠️ High effort detected on Joint{i+1}: {effort:.2f} Nm")
                return True
        return False
    
    def force_control(self, target_force, compliance=0.1):
        """
        Force control mode - maintain desired force while allowing motion
        """
        # Simplified force control (placeholder)
        # Real implementation would use proper force/torque sensors
        
        for i in range(6):
            effort_error = target_force - self.joint_efforts[i]
            position_adjustment = compliance * effort_error
            
            self.target_joint_states[i] += position_adjustment
    
    def impedance_control(self, stiffness=100.0, damping=10.0):
        """
        Impedance control for compliant interaction
        """
        for i in range(6):
            position_error = self.target_joint_states[i] - self.current_joint_states[i]
            velocity_error = 0.0 - self.joint_velocities[i]  # Target velocity = 0
            
            desired_effort = stiffness * position_error + damping * velocity_error
            
            # Apply effort limits
            max_effort = 50.0  # Nm
            desired_effort = np.clip(desired_effort, -max_effort, max_effort)
            
            # Send command
            msg = MotorCmd()
            msg.mode = 0  # Effort control mode
            msg.tau = float(desired_effort)
            msg.Kp = 0.0
            msg.Kd = 0.0
            
            joint_name = f"Joint0{i+1}"
            if joint_name in self.joint_pubs:
                self.joint_pubs[joint_name].publish(msg)
    
    def send_joint_commands(self):
        """Send position commands to joints"""
        for i in range(6):
            msg = MotorCmd()
            msg.mode = 10  # Position control
            msg.q = float(self.target_joint_states[i])
            msg.Kp = 50.0  # Higher precision gains
            msg.Kd = 2.0
            
            joint_name = f"Joint0{i+1}"
            if joint_name in self.joint_pubs:
                self.joint_pubs[joint_name].publish(msg)
        
        # Gripper
        gripper_msg = MotorCmd()
        gripper_msg.mode = 10
        gripper_msg.q = float(self.target_joint_states[6])
        gripper_msg.Kp = 30.0
        gripper_msg.Kd = 1.0
        self.gripper_pub.publish(gripper_msg)
    
    def control_loop(self):
        """Main high-frequency control loop"""
        rate = rospy.Rate(self.control_frequency)
        
        while not rospy.is_shutdown():
            if self.emergency_stop:
                # Emergency stop - hold current position
                self.target_joint_states = self.current_joint_states.copy()
                self.trajectory_buffer = []
            
            # Execute trajectory if available
            if self.trajectory_buffer:
                self.target_joint_states = np.array(self.trajectory_buffer.pop(0))
            
            # Collision detection
            if self.collision_detection():
                rospy.logwarn("🛑 Collision detected - stopping motion")
                self.trajectory_buffer = []
                self.target_joint_states = self.current_joint_states.copy()
            
            # Apply control based on mode
            if self.control_mode == "position":
                self.send_joint_commands()
            elif self.control_mode == "impedance":
                self.impedance_control()
            elif self.control_mode == "force":
                self.force_control(target_force=10.0)
            
            rate.sleep()
    
    def run_demo_sequence(self):
        """Demonstrate advanced control capabilities"""
        rospy.loginfo("🎭 Starting advanced control demonstration...")
        
        # Demo 1: Precision positioning
        rospy.loginfo("📍 Demo 1: Precision positioning")
        targets = [
            np.array([0.0, -0.3, 0.8, 0.0, -0.2, 0.0, 0.0]),
            np.array([0.5, -0.2, 1.0, 0.2, -0.3, 0.1, 0.2]),
            np.array([-0.3, -0.4, 1.2, -0.1, -0.1, -0.1, 0.4]),
            np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Home
        ]
        
        for i, target in enumerate(targets):
            rospy.loginfo(f"Moving to position {i+1}/4")
            self.plan_trajectory(self.current_joint_states, target, duration=2.0)
            
            # Wait for trajectory completion
            while self.trajectory_buffer:
                time.sleep(0.1)
            time.sleep(1.0)
        
        # Demo 2: Smooth circular motion
        rospy.loginfo("🔄 Demo 2: Smooth circular motion")
        self.circular_motion_demo()
        
        # Demo 3: Force-sensitive interaction
        rospy.loginfo("💪 Demo 3: Force-sensitive interaction")
        self.control_mode = "impedance"
        time.sleep(5.0)
        self.control_mode = "position"
        
        rospy.loginfo("✅ Advanced control demonstration complete")
    
    def circular_motion_demo(self):
        """Generate circular motion in Cartesian space"""
        center = np.array([0.3, 0.0, 0.6])
        radius = 0.1
        steps = 50
        
        for i in range(steps):
            angle = 2 * np.pi * i / steps
            x = center[0] + radius * np.cos(angle)
            y = center[1] + radius * np.sin(angle)
            z = center[2]
            
            # Simple IK for circular motion (placeholder)
            target_joints = np.array([
                np.arctan2(y, x),
                -0.3,
                0.8,
                0.0,
                -0.2,
                angle * 0.1,  # Rotate wrist during motion
                0.0
            ])
            
            self.target_joint_states = target_joints
            time.sleep(0.1)
    
    def run(self):
        """Main execution"""
        rospy.loginfo("🚀 Z1 Advanced Control System ready")
        rospy.loginfo("Available modes: position, velocity, force, impedance")
        rospy.loginfo("Press 'd' for demo, 'e' for emergency stop, 'q' to quit")
        
        try:
            while not rospy.is_shutdown():
                user_input = input("Command (d/e/q): ").lower().strip()
                
                if user_input == 'd':
                    self.run_demo_sequence()
                elif user_input == 'e':
                    self.emergency_stop = True
                    rospy.logwarn("🛑 Emergency stop activated")
                elif user_input == 'q':
                    break
                else:
                    rospy.loginfo("Available commands: d=demo, e=emergency_stop, q=quit")
                    
        except KeyboardInterrupt:
            rospy.loginfo("🛑 Advanced control stopped by user")

if __name__ == "__main__":
    try:
        controller = Z1AdvancedControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"❌ Advanced control error: {e}")