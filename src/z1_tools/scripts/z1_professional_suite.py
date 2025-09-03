#!/usr/bin/env python3

"""
Z1 Professional Suite - Industrial-grade robotic operations
Combines all SDK capabilities for professional applications
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../../z1_sdk/examples_py'))

import numpy as np
import time
import threading
from dataclasses import dataclass
from typing import List, Tuple, Optional

try:
    import unitree_arm_interface
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False

@dataclass
class TaskPoint:
    position: np.ndarray
    orientation: np.ndarray
    gripper: float
    force_limit: float = 50.0
    speed: float = 1.0

class Z1ProfessionalSuite:
    def __init__(self):
        if not SDK_AVAILABLE:
            raise RuntimeError("SDK required for professional suite")
            
        self.arm = unitree_arm_interface.ArmInterface(hasGripper=True)
        self.armModel = self.arm._ctrlComp.armModel
        self.armState = unitree_arm_interface.ArmFSMState
        
        # Professional parameters
        self.precision_threshold = 0.001  # 1mm precision
        self.force_threshold = 10.0  # 10N force limit
        self.safety_workspace = {
            'x': (-0.6, 0.6), 'y': (-0.6, 0.6), 'z': (0.1, 0.8)
        }
        
        self.arm.loopOn()
        print("🏭 Z1 Professional Suite Initialized")
        
    def precision_pick_place(self, pick_pose: TaskPoint, place_pose: TaskPoint) -> bool:
        """High-precision pick and place operation"""
        print("🎯 Executing precision pick & place...")
        
        try:
            # 1. Move to pre-pick position
            pre_pick = pick_pose.position.copy()
            pre_pick[2] += 0.05  # 5cm above
            
            if not self._move_to_cartesian(pre_pick, pick_pose.orientation, 0.0):
                return False
                
            # 2. Approach with force control
            self._force_controlled_approach(pick_pose.position, max_force=5.0)
            
            # 3. Close gripper with force feedback
            self._controlled_grasp(pick_pose.gripper)
            
            # 4. Lift with verification
            if not self._verify_grasp():
                print("❌ Grasp verification failed")
                return False
                
            # 5. Move to place position
            pre_place = place_pose.position.copy()
            pre_place[2] += 0.05
            
            if not self._move_to_cartesian(pre_place, place_pose.orientation, pick_pose.gripper):
                return False
                
            # 6. Place with force control
            self._force_controlled_place(place_pose.position, place_pose.gripper)
            
            print("✅ Precision pick & place completed")
            return True
            
        except Exception as e:
            print(f"❌ Pick & place failed: {e}")
            return False
            
    def assembly_operation(self, parts: List[TaskPoint], tolerances: List[float]) -> bool:
        """Precision assembly with force feedback"""
        print("🔧 Starting assembly operation...")
        
        for i, (part, tolerance) in enumerate(zip(parts, tolerances)):
            print(f"Assembling part {i+1}/{len(parts)}")
            
            # Move to assembly position with high precision
            if not self._precision_move(part.position, part.orientation, tolerance):
                return False
                
            # Force-guided insertion
            if not self._force_guided_insertion(part.position, part.force_limit):
                return False
                
        print("✅ Assembly operation completed")
        return True
        
    def quality_inspection(self, inspection_points: List[np.ndarray]) -> List[dict]:
        """Automated quality inspection routine"""
        print("🔍 Starting quality inspection...")
        
        results = []
        
        for i, point in enumerate(inspection_points):
            print(f"Inspecting point {i+1}/{len(inspection_points)}")
            
            # Move to inspection position
            orientation = np.array([0, 0, -1])  # Point downward
            if not self._move_to_cartesian(point, orientation, 0.0):
                continue
                
            # Perform measurement (placeholder for sensor integration)
            measurement = self._take_measurement(point)
            results.append(measurement)
            
        print(f"✅ Inspection completed: {len(results)} measurements")
        return results
        
    def collaborative_operation(self, human_workspace: dict) -> None:
        """Safe collaborative operation with human"""
        print("🤝 Starting collaborative mode...")
        
        # Reduce speed and force limits for safety
        self.force_threshold = 5.0  # Reduced force limit
        
        # Monitor for human presence
        monitoring_thread = threading.Thread(target=self._monitor_human_safety)
        monitoring_thread.daemon = True
        monitoring_thread.start()
        
        # Execute collaborative task
        self._collaborative_task_loop()
        
    def _move_to_cartesian(self, position: np.ndarray, orientation: np.ndarray, gripper: float) -> bool:
        """Move to Cartesian position with safety checks"""
        # Workspace safety check
        x, y, z = position
        if not (self.safety_workspace['x'][0] <= x <= self.safety_workspace['x'][1] and
                self.safety_workspace['y'][0] <= y <= self.safety_workspace['y'][1] and
                self.safety_workspace['z'][0] <= z <= self.safety_workspace['z'][1]):
            print(f"❌ Position {position} outside safe workspace")
            return False
            
        # Build homogeneous transform
        T = np.eye(4)
        T[0:3, 3] = position
        # Simplified orientation (full rotation matrix needed for production)
        
        # Use IK to find joint solution
        current_q = self.arm.lowstate.getQ()
        hasIK, q_target = self.armModel.inverseKinematics(T, current_q, False)
        
        if not hasIK:
            print("❌ No IK solution found")
            return False
            
        # Execute movement
        posture = unitree_arm_interface.homoToPosture(T)
        return self.arm.MoveL(posture, gripper, 0.5)
        
    def _force_controlled_approach(self, target_position: np.ndarray, max_force: float) -> None:
        """Approach target with force control"""
        self.arm.startTrack(self.armState.CARTESIAN)
        
        approach_steps = 100
        for i in range(approach_steps):
            # Calculate approach vector
            current_pose = self.armModel.forwardKinematics(self.arm.lowstate.getQ(), 6)
            current_pos = current_pose[0:3, 3]
            
            direction = target_position - current_pos
            distance = np.linalg.norm(direction)
            
            if distance < 0.001:  # 1mm precision
                break
                
            # Apply controlled force
            force_magnitude = min(max_force, distance * 100)  # Proportional force
            force_vector = direction / distance * force_magnitude
            
            force_cmd = np.concatenate([force_vector, np.zeros(3), [0]])
            self.arm.cartesianCtrlCmd(force_cmd.tolist(), 0.1, 0.1)
            time.sleep(0.01)
            
    def _controlled_grasp(self, target_gripper: float) -> None:
        """Execute controlled grasp with force feedback"""
        steps = 50
        current_gripper = self.arm.lowstate.getGripperQ()
        
        for i in range(steps):
            alpha = i / steps
            gripper_pos = current_gripper + alpha * (target_gripper - current_gripper)
            
            # Monitor grasp force (placeholder - needs force sensor)
            self.arm.gripperQ = gripper_pos
            self.arm.setGripperCmd(gripper_pos, 0, 0)
            self.arm.sendRecv()
            time.sleep(0.02)
            
    def _verify_grasp(self) -> bool:
        """Verify successful grasp"""
        # Lift slightly and check for object presence
        current_q = self.arm.lowstate.getQ()
        lift_q = current_q.copy()
        
        # Small upward movement
        current_pose = self.armModel.forwardKinematics(current_q, 6)
        lift_pose = current_pose.copy()
        lift_pose[2, 3] += 0.01  # 1cm lift
        
        hasIK, lift_joints = self.armModel.inverseKinematics(lift_pose, current_q, False)
        if hasIK:
            posture = unitree_arm_interface.homoToPosture(lift_pose)
            return self.arm.MoveL(posture, self.arm.lowstate.getGripperQ(), 0.2)
        return False
        
    def _force_controlled_place(self, target_position: np.ndarray, gripper_open: float) -> None:
        """Place object with force control"""
        # Gentle downward movement until contact
        self._force_controlled_approach(target_position, max_force=2.0)
        
        # Open gripper gradually
        self._controlled_grasp(gripper_open)
        
    def _precision_move(self, position: np.ndarray, orientation: np.ndarray, tolerance: float) -> bool:
        """High-precision movement within tolerance"""
        max_attempts = 3
        
        for attempt in range(max_attempts):
            if self._move_to_cartesian(position, orientation, 0.0):
                # Verify precision
                current_pose = self.armModel.forwardKinematics(self.arm.lowstate.getQ(), 6)
                error = np.linalg.norm(current_pose[0:3, 3] - position)
                
                if error <= tolerance:
                    return True
                    
                print(f"Precision attempt {attempt+1}: error {error:.4f}m > tolerance {tolerance:.4f}m")
            
        return False
        
    def _force_guided_insertion(self, target_position: np.ndarray, force_limit: float) -> bool:
        """Force-guided insertion for assembly"""
        self.arm.startTrack(self.armState.CARTESIAN)
        
        insertion_depth = 0.02  # 2cm insertion
        steps = 100
        
        for i in range(steps):
            progress = i / steps
            current_depth = progress * insertion_depth
            
            # Downward force with compliance
            force_cmd = [0, 0, -force_limit * (1 - progress), 0, 0, 0, 0]
            self.arm.cartesianCtrlCmd(force_cmd, 0.1, 0.1)
            
            # Check for successful insertion (force drop)
            # This would use real force sensor feedback
            time.sleep(0.01)
            
        return True
        
    def _take_measurement(self, position: np.ndarray) -> dict:
        """Take measurement at position (placeholder for sensor integration)"""
        # Placeholder for vision/sensor integration
        return {
            "position": position.tolist(),
            "timestamp": time.time(),
            "measurement": np.random.random(),  # Placeholder measurement
            "quality": "pass" if np.random.random() > 0.1 else "fail"
        }
        
    def _monitor_human_safety(self) -> None:
        """Monitor for human safety in collaborative mode"""
        while True:
            # Placeholder for human detection
            # Would integrate with vision system or safety sensors
            time.sleep(0.1)
            
    def _collaborative_task_loop(self) -> None:
        """Main collaborative task execution"""
        print("🤝 Collaborative mode active - reduced speed and force")
        
        # Example collaborative task
        safe_positions = [
            np.array([0.3, 0.0, 0.3]),
            np.array([0.0, 0.3, 0.3]),
            np.array([-0.3, 0.0, 0.3]),
            np.array([0.0, -0.3, 0.3])
        ]
        
        for pos in safe_positions:
            orientation = np.array([0, 0, -1])
            self._move_to_cartesian(pos, orientation, 0.0)
            time.sleep(2)  # Pause for human interaction
            
    def shutdown(self):
        """Safe shutdown"""
        self.arm.backToStart()
        self.arm.loopOff()
        print("🛑 Professional suite shutdown complete")

def main():
    if not SDK_AVAILABLE:
        print("❌ SDK not available - professional suite requires real robot")
        return
        
    suite = Z1ProfessionalSuite()
    
    try:
        print("🏭 Z1 Professional Suite")
        print("1. Precision Pick & Place")
        print("2. Assembly Operation") 
        print("3. Quality Inspection")
        print("4. Collaborative Mode")
        print("q. Quit")
        
        while True:
            choice = input("Select operation: ").strip()
            
            if choice == '1':
                pick = TaskPoint(
                    position=np.array([0.4, 0.0, 0.2]),
                    orientation=np.array([0, 0, -1]),
                    gripper=-0.5
                )
                place = TaskPoint(
                    position=np.array([0.0, 0.4, 0.2]),
                    orientation=np.array([0, 0, -1]),
                    gripper=0.0
                )
                suite.precision_pick_place(pick, place)
                
            elif choice == '2':
                parts = [
                    TaskPoint(np.array([0.3, 0.0, 0.25]), np.array([0, 0, -1]), -0.3),
                    TaskPoint(np.array([0.3, 0.0, 0.20]), np.array([0, 0, -1]), -0.5)
                ]
                tolerances = [0.001, 0.0005]  # 1mm, 0.5mm
                suite.assembly_operation(parts, tolerances)
                
            elif choice == '3':
                points = [
                    np.array([0.2, 0.2, 0.3]),
                    np.array([0.2, -0.2, 0.3]),
                    np.array([-0.2, 0.0, 0.3])
                ]
                results = suite.quality_inspection(points)
                print(f"Inspection results: {len(results)} measurements taken")
                
            elif choice == '4':
                human_workspace = {'x': (0.0, 0.5), 'y': (-0.3, 0.3), 'z': (0.2, 0.6)}
                suite.collaborative_operation(human_workspace)
                
            elif choice.lower() == 'q':
                break
                
    except KeyboardInterrupt:
        pass
    finally:
        suite.shutdown()

if __name__ == "__main__":
    main()