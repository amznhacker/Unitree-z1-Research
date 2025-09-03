#!/usr/bin/env python3

"""
Z1 Pure SDK Control - No ROS Dependencies
Direct hardware control for production environments
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../../z1_sdk/examples_py'))

import numpy as np
import time
import threading
from dataclasses import dataclass
from typing import List, Optional

try:
    import unitree_arm_interface
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False
    print("❌ SDK not available")
    sys.exit(1)

@dataclass
class RobotState:
    joints: np.ndarray
    position: np.ndarray
    gripper: float
    forces: np.ndarray

class Z1PureSDKControl:
    def __init__(self):
        self.arm = unitree_arm_interface.ArmInterface(hasGripper=True)
        self.armModel = self.arm._ctrlComp.armModel
        self.armState = unitree_arm_interface.ArmFSMState
        
        self.running = False
        self.current_state = RobotState(
            joints=np.zeros(6),
            position=np.zeros(3),
            gripper=0.0,
            forces=np.zeros(6)
        )
        
        print("🤖 Pure SDK Control Initialized")
        
    def start(self):
        """Start control loop"""
        self.arm.loopOn()
        self.running = True
        
        # Start state monitoring thread
        self.monitor_thread = threading.Thread(target=self._monitor_loop)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        
    def stop(self):
        """Stop control and return to safe position"""
        self.running = False
        self.arm.backToStart()
        self.arm.loopOff()
        
    def get_state(self) -> RobotState:
        """Get current robot state"""
        q = self.arm.lowstate.getQ()
        T = self.armModel.forwardKinematics(q, 6)
        
        self.current_state.joints = q
        self.current_state.position = T[0:3, 3]
        self.current_state.gripper = self.arm.lowstate.getGripperQ()
        
        return self.current_state
        
    def move_joints(self, target_joints: List[float], speed: float = 1.0) -> bool:
        """Move to joint configuration"""
        try:
            T = self.armModel.forwardKinematics(np.array(target_joints), 6)
            posture = unitree_arm_interface.homoToPosture(T)
            return self.arm.MoveJ(posture, self.current_state.gripper, speed)
        except Exception as e:
            print(f"❌ Joint move failed: {e}")
            return False
            
    def move_cartesian(self, target_pos: List[float], speed: float = 0.5) -> bool:
        """Move to Cartesian position"""
        try:
            return self.arm.MoveL(target_pos, self.current_state.gripper, speed)
        except Exception as e:
            print(f"❌ Cartesian move failed: {e}")
            return False
            
    def set_gripper(self, position: float) -> None:
        """Set gripper position (-1.0 to 1.0)"""
        self.arm.gripperQ = position
        
    def force_control(self, forces: List[float], duration: float = 1.0) -> None:
        """Apply force control"""
        self.arm.startTrack(self.armState.CARTESIAN)
        
        steps = int(duration / 0.002)
        for _ in range(steps):
            self.arm.cartesianCtrlCmd(forces + [0], 0.1, 0.1)
            time.sleep(0.002)
            
    def low_level_control(self, q: np.ndarray, qd: np.ndarray, tau: np.ndarray) -> None:
        """Direct low-level control"""
        self.arm.setFsmLowcmd()
        
        self.arm.q = q
        self.arm.qd = qd
        self.arm.tau = tau
        
        self.arm.setArmCmd(q, qd, tau)
        self.arm.sendRecv()
        
    def gravity_compensation_mode(self) -> None:
        """Enable gravity compensation"""
        self.arm.setFsmLowcmd()
        
        while self.running:
            q = self.arm.lowstate.getQ()
            tau_gravity = self.armModel.inverseDynamics(q, np.zeros(6), np.zeros(6), np.zeros(6))
            
            self.arm.q = q
            self.arm.qd = np.zeros(6)
            self.arm.tau = tau_gravity
            
            self.arm.setArmCmd(self.arm.q, self.arm.qd, self.arm.tau)
            self.arm.sendRecv()
            time.sleep(0.002)
            
    def _monitor_loop(self):
        """Background monitoring"""
        while self.running:
            self.get_state()
            time.sleep(0.01)

# Simple control interface
def main():
    controller = Z1PureSDKControl()
    controller.start()
    
    try:
        print("🎮 Pure SDK Control Interface")
        print("Commands: j=joints, c=cartesian, g=gripper, f=force, l=low-level, q=quit")
        
        while True:
            cmd = input("Command: ").lower().strip()
            
            if cmd == 'j':
                joints = [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]
                controller.move_joints(joints)
                
            elif cmd == 'c':
                pos = [0.3, 0.0, 0.4, 0.0, 0.0, 0.0]
                controller.move_cartesian(pos)
                
            elif cmd == 'g':
                controller.set_gripper(-0.5)
                
            elif cmd == 'f':
                forces = [0, 0, -5, 0, 0, 0]  # 5N downward
                controller.force_control(forces, 2.0)
                
            elif cmd == 'l':
                print("Starting gravity compensation...")
                controller.gravity_compensation_mode()
                
            elif cmd == 'q':
                break
                
            # Show current state
            state = controller.get_state()
            print(f"Position: {state.position}")
            print(f"Joints: {state.joints}")
            
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()

if __name__ == "__main__":
    main()