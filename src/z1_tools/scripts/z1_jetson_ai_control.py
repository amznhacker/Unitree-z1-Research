#!/usr/bin/env python3

"""
Z1 Jetson AI Control - Autonomous robot with computer vision
Combines Z1 SDK with Jetson AI capabilities
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../../z1_sdk/examples_py'))

import numpy as np
import cv2
import time
import threading
from dataclasses import dataclass
from typing import List, Tuple, Optional

# Jetson-specific imports
try:
    import jetson.inference
    import jetson.utils
    JETSON_AVAILABLE = True
except ImportError:
    JETSON_AVAILABLE = False
    print("⚠️ Jetson inference not available - using OpenCV fallback")

try:
    import unitree_arm_interface
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False

@dataclass
class DetectedObject:
    class_id: int
    confidence: float
    bbox: Tuple[int, int, int, int]  # x, y, w, h
    center_3d: Optional[np.ndarray] = None

class Z1JetsonAIControl:
    def __init__(self, camera_id=0):
        if not SDK_AVAILABLE:
            raise RuntimeError("Z1 SDK required")
            
        # Initialize robot
        self.arm = unitree_arm_interface.ArmInterface(hasGripper=True)
        self.armModel = self.arm._ctrlComp.armModel
        
        # Initialize AI vision
        if JETSON_AVAILABLE:
            self.net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.5)
            self.camera = jetson.utils.videoSource(f"csi://{camera_id}")
            self.display = jetson.utils.videoOutput("display://0")
        else:
            # OpenCV fallback
            self.cap = cv2.VideoCapture(camera_id)
            
        # Camera calibration (example values - calibrate for your setup)
        self.camera_matrix = np.array([
            [800, 0, 320],
            [0, 800, 240],
            [0, 0, 1]
        ], dtype=np.float32)
        
        # Robot-camera transformation (hand-eye calibration needed)
        self.camera_to_robot = np.eye(4)
        self.camera_to_robot[0:3, 3] = [0.1, 0.0, 0.05]  # Camera offset from end-effector
        
        self.running = False
        self.detected_objects = []
        
        print("🤖🎥 Jetson AI Control Initialized")
        
    def start(self):
        """Start robot and vision systems"""
        self.arm.loopOn()
        self.running = True
        
        # Start vision thread
        self.vision_thread = threading.Thread(target=self._vision_loop)
        self.vision_thread.daemon = True
        self.vision_thread.start()
        
    def stop(self):
        """Stop all systems"""
        self.running = False
        self.arm.backToStart()
        self.arm.loopOff()
        
        if not JETSON_AVAILABLE:
            self.cap.release()
        cv2.destroyAllWindows()
        
    def _vision_loop(self):
        """Main vision processing loop"""
        while self.running:
            if JETSON_AVAILABLE:
                self._jetson_vision_process()
            else:
                self._opencv_vision_process()
            time.sleep(0.033)  # ~30 FPS
            
    def _jetson_vision_process(self):
        """Jetson-optimized vision processing"""
        img = self.camera.Capture()
        if img is None:
            return
            
        # AI object detection
        detections = self.net.Detect(img)
        
        # Convert to our format
        self.detected_objects = []
        for detection in detections:
            obj = DetectedObject(
                class_id=detection.ClassID,
                confidence=detection.Confidence,
                bbox=(int(detection.Left), int(detection.Top), 
                     int(detection.Width), int(detection.Height))
            )
            
            # Estimate 3D position
            obj.center_3d = self._estimate_3d_position(obj.bbox)
            self.detected_objects.append(obj)
            
        # Render and display
        self.display.Render(img)
        
    def _opencv_vision_process(self):
        """OpenCV fallback vision processing"""
        ret, frame = self.cap.read()
        if not ret:
            return
            
        # Simple color-based detection (placeholder)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Detect red objects (example)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        self.detected_objects = []
        for contour in contours:
            if cv2.contourArea(contour) > 500:
                x, y, w, h = cv2.boundingRect(contour)
                obj = DetectedObject(
                    class_id=1,  # Red object
                    confidence=0.8,
                    bbox=(x, y, w, h)
                )
                obj.center_3d = self._estimate_3d_position(obj.bbox)
                self.detected_objects.append(obj)
                
                # Draw detection
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                
        cv2.imshow("Z1 Vision", frame)
        cv2.waitKey(1)
        
    def _estimate_3d_position(self, bbox: Tuple[int, int, int, int]) -> np.ndarray:
        """Estimate 3D position from 2D bounding box"""
        x, y, w, h = bbox
        center_x = x + w // 2
        center_y = y + h // 2
        
        # Assume object is on table at known height
        # Real implementation would use stereo vision or depth camera
        table_height = 0.0  # meters
        
        # Simple pinhole camera model
        # Real implementation needs proper camera calibration
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
        
        # Estimate depth based on object size (very approximate)
        estimated_depth = 0.5  # 50cm default
        
        # Convert to 3D camera coordinates
        x_cam = (center_x - cx) * estimated_depth / fx
        y_cam = (center_y - cy) * estimated_depth / fy
        z_cam = estimated_depth
        
        # Transform to robot coordinates
        pos_camera = np.array([x_cam, y_cam, z_cam, 1])
        pos_robot = self.camera_to_robot @ pos_camera
        
        return pos_robot[0:3]
        
    def autonomous_pick_and_place(self):
        """Autonomous pick and place based on vision"""
        print("🎯 Starting autonomous pick and place...")
        
        # Wait for object detection
        while not self.detected_objects and self.running:
            print("👀 Waiting for objects...")
            time.sleep(1)
            
        if not self.detected_objects:
            return False
            
        # Select highest confidence object
        target_obj = max(self.detected_objects, key=lambda x: x.confidence)
        print(f"🎯 Target: Class {target_obj.class_id}, Confidence: {target_obj.confidence:.2f}")
        
        if target_obj.center_3d is None:
            print("❌ No 3D position available")
            return False
            
        # Move to pre-pick position
        pre_pick = target_obj.center_3d.copy()
        pre_pick[2] += 0.1  # 10cm above
        
        print("📍 Moving to pre-pick position...")
        if not self._move_to_position(pre_pick):
            return False
            
        # Approach object
        print("⬇️ Approaching object...")
        if not self._move_to_position(target_obj.center_3d):
            return False
            
        # Close gripper
        print("🤏 Grasping object...")
        self.arm.gripperQ = -0.8
        time.sleep(1)
        
        # Lift object
        lift_pos = target_obj.center_3d.copy()
        lift_pos[2] += 0.1
        if not self._move_to_position(lift_pos):
            return False
            
        # Move to place position (example)
        place_pos = np.array([0.0, 0.4, 0.2])
        print("📦 Moving to place position...")
        if not self._move_to_position(place_pos):
            return False
            
        # Release object
        print("🤲 Releasing object...")
        self.arm.gripperQ = 0.0
        time.sleep(1)
        
        print("✅ Autonomous pick and place completed!")
        return True
        
    def _move_to_position(self, position: np.ndarray) -> bool:
        """Move robot to 3D position"""
        try:
            # Build target pose
            target_pose = [position[0], position[1], position[2], 0, 0, 0]
            return self.arm.MoveL(target_pose, self.arm.lowstate.getGripperQ(), 0.3)
        except Exception as e:
            print(f"❌ Move failed: {e}")
            return False
            
    def visual_servoing(self, target_class_id: int):
        """Visual servoing to keep object in center of view"""
        print(f"👁️ Starting visual servoing for class {target_class_id}")
        
        while self.running:
            # Find target object
            target_objects = [obj for obj in self.detected_objects 
                            if obj.class_id == target_class_id]
            
            if not target_objects:
                time.sleep(0.1)
                continue
                
            target = target_objects[0]
            
            # Calculate error from image center
            img_center_x, img_center_y = 320, 240  # Assume 640x480 image
            obj_center_x = target.bbox[0] + target.bbox[2] // 2
            obj_center_y = target.bbox[1] + target.bbox[3] // 2
            
            error_x = obj_center_x - img_center_x
            error_y = obj_center_y - img_center_y
            
            # Convert to robot motion (simplified)
            if abs(error_x) > 20 or abs(error_y) > 20:
                # Calculate small adjustment
                dx = -error_x * 0.0001  # Scale factor
                dy = -error_y * 0.0001
                
                # Get current position and adjust
                current_q = self.arm.lowstate.getQ()
                current_pose = self.armModel.forwardKinematics(current_q, 6)
                
                new_pos = current_pose[0:3, 3] + np.array([dx, dy, 0])
                target_pose = [new_pos[0], new_pos[1], new_pos[2], 0, 0, 0]
                
                self.arm.MoveL(target_pose, self.arm.lowstate.getGripperQ(), 0.1)
                
            time.sleep(0.1)

def main():
    try:
        controller = Z1JetsonAIControl()
        controller.start()
        
        print("🤖🎥 Jetson AI Control Active")
        print("Commands:")
        print("  a = Autonomous pick & place")
        print("  v = Visual servoing (class 1)")
        print("  s = Show detections")
        print("  q = Quit")
        
        while True:
            cmd = input("Command: ").lower().strip()
            
            if cmd == 'a':
                controller.autonomous_pick_and_place()
            elif cmd == 'v':
                controller.visual_servoing(1)
            elif cmd == 's':
                print(f"Detected {len(controller.detected_objects)} objects:")
                for i, obj in enumerate(controller.detected_objects):
                    print(f"  {i}: Class {obj.class_id}, Conf: {obj.confidence:.2f}")
            elif cmd == 'q':
                break
                
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()

if __name__ == "__main__":
    main()