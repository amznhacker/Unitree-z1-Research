#!/usr/bin/env python3

"""
Z1 SDK Enhanced Control - Full SDK Integration
Leverages: IK/FK, Dynamics, FSM, High/Low-level control, Force control
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../../z1_sdk/examples_py'))

import rospy
import numpy as np
import threading
import time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

try:
    import unitree_arm_interface
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False
    print("SDK not available - simulation mode only")

class Z1SDKEnhancedControl:
    def __init__(self):
        rospy.init_node("z1_sdk_enhanced_control")
        
        self.sdk_mode = SDK_AVAILABLE
        if self.sdk_mode:
            self.arm = unitree_arm_interface.ArmInterface(hasGripper=True)
            self.armModel = self.arm._ctrlComp.armModel
            self.armState = unitree_arm_interface.ArmFSMState
            
        # Control modes
        self.control_modes = ["joint", "cartesian", "force", "impedance", "gravity_comp"]
        self.current_mode = "joint"
        
        # State variables
        self.current_q = np.zeros(6)
        self.target_pose = np.eye(4)
        self.force_target = np.zeros(6)
        
        # Publishers for simulation
        self.joint_pubs = {}
        for i in range(1, 7):
            topic = f"/z1_gazebo/Joint0{i}_controller/command"
            self.joint_pubs[f"Joint0{i}"] = rospy.Publisher(topic, Float64MultiArray, queue_size=1)
        
        # Subscribers
        rospy.Subscriber("/z1_enhanced/target_pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/z1_enhanced/mode", String, self.mode_callback)
        rospy.Subscriber("/z1_enhanced/force_target", Float64MultiArray, self.force_callback)
        
        if self.sdk_mode:
            self.arm.loopOn()
            
        rospy.loginfo("🚀 Z1 SDK Enhanced Control Ready")
        
    def pose_callback(self, msg):
        """Convert ROS pose to homogeneous transform"""
        T = np.eye(4)
        T[0:3, 3] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        # Simplified rotation (full quaternion conversion needed for production)
        self.target_pose = T
        
    def mode_callback(self, msg):
        if msg.data in self.control_modes:
            self.current_mode = msg.data
            rospy.loginfo(f"Switched to {self.current_mode} mode")
            
    def force_callback(self, msg):
        self.force_target = np.array(msg.data[:6])
        
    def sdk_inverse_kinematics(self, target_pose):
        """Use SDK's IK solver"""
        if not self.sdk_mode:
            return None
            
        hasIK, q_result = self.armModel.inverseKinematics(target_pose, self.current_q, False)
        return q_result if hasIK else None
        
    def sdk_forward_kinematics(self, q):
        """Use SDK's FK solver"""
        if not self.sdk_mode:
            return np.eye(4)
        return self.armModel.forwardKinematics(q, 6)
        
    def gravity_compensation(self, q):
        """Calculate gravity compensation torques"""
        if not self.sdk_mode:
            return np.zeros(6)
        return self.armModel.inverseDynamics(q, np.zeros(6), np.zeros(6), np.zeros(6))
        
    def jacobian_control(self, spatial_twist, q_current):
        """Use SDK's Jacobian and QP solver"""
        if not self.sdk_mode:
            return np.zeros(6)
        return self.armModel.solveQP(spatial_twist, q_current, self.arm._ctrlComp.dt)
        
    def execute_move_commands(self):
        """Execute high-level SDK move commands"""
        if not self.sdk_mode:
            return
            
        # Demo sequence using SDK high-level commands
        rospy.loginfo("Executing SDK MoveJ/MoveL/MoveC sequence")
        
        # MoveJ - Joint space movement
        forward_posture = [0, 0, 0, 0.45, -0.2, 0.2]
        self.arm.MoveJ(forward_posture, 0.0, 2.0)
        
        # MoveL - Linear movement
        target_posture = [0, 0, 0, 0.45, 0.2, 0.2]
        self.arm.MoveL(target_posture, -1.0, 1.0)
        
        # MoveC - Circular movement
        middle_posture = [0, 0, 0, 0.45, 0, 0.4]
        end_posture = [0, 0, 0, 0.45, -0.2, 0.2]
        self.arm.MoveC(middle_posture, end_posture, 0.0, 1.0)
        
    def low_level_control_demo(self):
        """Demonstrate low-level torque control"""
        if not self.sdk_mode:
            return
            
        rospy.loginfo("Starting low-level control demo")
        self.arm.setFsmLowcmd()
        
        duration = 500
        start_q = self.arm.lowstate.getQ()
        target_q = np.array([0.0, 1.0, -0.5, 0.0, 0.0, 0.0])
        
        for i in range(duration):
            # Interpolate position
            alpha = i / duration
            q_des = start_q * (1 - alpha) + target_q * alpha
            qd_des = (target_q - start_q) / (duration * 0.002)
            
            # Calculate required torques using inverse dynamics
            tau = self.armModel.inverseDynamics(q_des, qd_des, np.zeros(6), np.zeros(6))
            
            # Send low-level commands
            self.arm.q = q_des
            self.arm.qd = qd_des
            self.arm.tau = tau
            self.arm.gripperQ = -alpha
            
            self.arm.setArmCmd(self.arm.q, self.arm.qd, self.arm.tau)
            self.arm.setGripperCmd(self.arm.gripperQ, 0, 0)
            self.arm.sendRecv()
            
            time.sleep(0.002)
            
    def force_control_demo(self):
        """Demonstrate force-sensitive control"""
        if not self.sdk_mode:
            return
            
        rospy.loginfo("Starting force control demo")
        self.arm.startTrack(self.armState.CARTESIAN)
        
        # Apply gentle downward force
        for i in range(1000):
            # Force in negative Z direction
            force_cmd = [0, 0, 0, 0, 0, -5.0, 0]  # 5N downward
            self.arm.cartesianCtrlCmd(force_cmd, 0.1, 0.1)
            time.sleep(self.arm._ctrlComp.dt)
            
    def impedance_control_demo(self):
        """Demonstrate compliant interaction"""
        if not self.sdk_mode:
            return
            
        rospy.loginfo("Starting impedance control demo")
        
        # Set compliant parameters
        stiffness = np.diag([100, 100, 100, 10, 10, 10])  # Lower stiffness for compliance
        damping = np.diag([20, 20, 20, 2, 2, 2])
        
        target_pose = self.sdk_forward_kinematics(self.current_q)
        
        for i in range(1000):
            current_pose = self.sdk_forward_kinematics(self.current_q)
            
            # Calculate pose error (simplified)
            pos_error = target_pose[0:3, 3] - current_pose[0:3, 3]
            
            # Calculate desired force
            desired_force = stiffness[0:3, 0:3] @ pos_error
            
            # Apply force command
            force_cmd = np.concatenate([desired_force, np.zeros(3), [0]])
            self.arm.cartesianCtrlCmd(force_cmd.tolist(), 0.1, 0.1)
            time.sleep(self.arm._ctrlComp.dt)
            
    def run_comprehensive_demo(self):
        """Run all SDK capabilities"""
        rospy.loginfo("🎯 Starting comprehensive SDK demo")
        
        demos = [
            ("High-level Commands", self.execute_move_commands),
            ("Low-level Control", self.low_level_control_demo),
            ("Force Control", self.force_control_demo),
            ("Impedance Control", self.impedance_control_demo)
        ]
        
        for name, demo_func in demos:
            rospy.loginfo(f"Running {name} demo...")
            try:
                demo_func()
                rospy.loginfo(f"✅ {name} demo completed")
            except Exception as e:
                rospy.logerr(f"❌ {name} demo failed: {e}")
            
            time.sleep(2)  # Pause between demos
            
        if self.sdk_mode:
            self.arm.backToStart()
            
    def run(self):
        """Main execution"""
        rospy.loginfo("Z1 SDK Enhanced Control System")
        rospy.loginfo("Commands: 'd'=demo, 'h'=high-level, 'l'=low-level, 'f'=force, 'i'=impedance, 'q'=quit")
        
        try:
            while not rospy.is_shutdown():
                cmd = input("Command: ").lower().strip()
                
                if cmd == 'd':
                    self.run_comprehensive_demo()
                elif cmd == 'h':
                    self.execute_move_commands()
                elif cmd == 'l':
                    self.low_level_control_demo()
                elif cmd == 'f':
                    self.force_control_demo()
                elif cmd == 'i':
                    self.impedance_control_demo()
                elif cmd == 'q':
                    break
                    
        except KeyboardInterrupt:
            pass
        finally:
            if self.sdk_mode:
                self.arm.loopOff()

if __name__ == "__main__":
    controller = Z1SDKEnhancedControl()
    controller.run()