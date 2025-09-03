#!/usr/bin/env python3

"""
Z1 Computer Vision - Object detection and visual servoing for Z1 robotic arm
Features: Real-time object detection, visual tracking, automated pick-and-place
"""

import rospy
import cv2
import numpy as np
import threading
import time
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
from unitree_legged_msgs.msg import MotorCmd
import json

class Z1ComputerVision:
    def __init__(self):
        rospy.init_node("z1_computer_vision")
        
        # OpenCV setup
        self.bridge = CvBridge()
        self.current_frame = None
        self.detection_active = True
        
        # Object detection setup
        self.net = None
        self.output_layers = None
        self.classes = []
        self.colors = np.random.uniform(0, 255, size=(100, 3))
        
        # Load YOLO model (if available)
        self.load_yolo_model()
        
        # Robot control
        self.joint_pubs = {}
        for i in range(1, 7):
            topic = f"/z1_gazebo/Joint0{i}_controller/command"
            self.joint_pubs[f"Joint0{i}"] = rospy.Publisher(topic, MotorCmd, queue_size=1)
        
        self.gripper_pub = rospy.Publisher("/z1_gazebo/Gripper_controller/command", MotorCmd, queue_size=1)
        
        # Publishers for detected objects
        self.detection_pub = rospy.Publisher("/z1_vision/detections", String, queue_size=10)
        self.target_pub = rospy.Publisher("/z1_vision/target_point", PointStamped, queue_size=10)
        self.image_pub = rospy.Publisher("/z1_vision/annotated_image", Image, queue_size=1)
        
        # Subscribers
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        rospy.Subscriber("/z1_vision/pick_object", String, self.pick_object_callback)
        
        # Camera parameters (calibration needed for real camera)
        self.camera_matrix = np.array([[800, 0, 320],
                                     [0, 800, 240],
                                     [0, 0, 1]], dtype=np.float32)
        
        # Current robot state
        self.current_positions = {f"Joint0{i}": 0.0 for i in range(1, 7)}
        self.current_positions["Gripper"] = 0.0
        
        # Visual servoing parameters
        self.target_object = None
        self.servoing_active = False
        
        rospy.loginfo("🎥 Z1 Computer Vision System initialized")
        rospy.loginfo("📹 Waiting for camera feed...")
        
    def load_yolo_model(self):
        """Load YOLO object detection model"""
        try:
            # Try to load YOLO weights (user needs to download these)
            weights_path = "/opt/yolo/yolov3.weights"
            config_path = "/opt/yolo/yolov3.cfg"
            classes_path = "/opt/yolo/coco.names"
            
            if all(os.path.exists(p) for p in [weights_path, config_path, classes_path]):
                self.net = cv2.dnn.readNet(weights_path, config_path)
                layer_names = self.net.getLayerNames()
                self.output_layers = [layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
                
                with open(classes_path, "r") as f:
                    self.classes = [line.strip() for line in f.readlines()]
                
                rospy.loginfo("✅ YOLO model loaded successfully")
            else:
                rospy.logwarn("⚠️ YOLO model not found - using basic detection")
                self.use_basic_detection()
                
        except Exception as e:
            rospy.logwarn(f"⚠️ Could not load YOLO: {e} - using basic detection")
            self.use_basic_detection()
    
    def use_basic_detection(self):
        """Fallback to basic color-based detection"""
        self.classes = ["red_object", "blue_object", "green_object"]
        rospy.loginfo("📦 Using basic color-based object detection")
    
    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            if self.detection_active:
                self.process_frame()
                
        except Exception as e:
            rospy.logerr(f"Image processing error: {e}")
    
    def process_frame(self):
        """Process frame for object detection"""
        if self.current_frame is None:
            return
        
        frame = self.current_frame.copy()
        
        if self.net is not None:
            # YOLO detection
            detections = self.yolo_detect(frame)
        else:
            # Basic color detection
            detections = self.color_detect(frame)
        
        # Annotate frame
        annotated_frame = self.annotate_frame(frame, detections)
        
        # Publish results
        self.publish_detections(detections)
        self.publish_annotated_image(annotated_frame)
        
        # Visual servoing
        if self.servoing_active and self.target_object:
            self.visual_servo(detections)
    
    def yolo_detect(self, frame):
        """YOLO-based object detection"""
        height, width, channels = frame.shape
        
        # Prepare input
        blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outputs = self.net.forward(self.output_layers)
        
        # Parse detections
        boxes = []
        confidences = []
        class_ids = []
        
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                
                if confidence > 0.5:  # Confidence threshold
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
        
        # Non-maximum suppression
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        
        detections = []
        if len(indexes) > 0:
            for i in indexes.flatten():
                x, y, w, h = boxes[i]
                class_name = self.classes[class_ids[i]]
                confidence = confidences[i]
                
                detections.append({
                    'class': class_name,
                    'confidence': confidence,
                    'bbox': [x, y, w, h],
                    'center': [x + w//2, y + h//2]
                })
        
        return detections
    
    def color_detect(self, frame):
        """Basic color-based object detection"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Color ranges (HSV)
        color_ranges = {
            'red_object': [(0, 50, 50), (10, 255, 255)],
            'blue_object': [(100, 50, 50), (130, 255, 255)],
            'green_object': [(40, 50, 50), (80, 255, 255)]
        }
        
        detections = []
        
        for color_name, (lower, upper) in color_ranges.items():
            lower = np.array(lower)
            upper = np.array(upper)
            
            mask = cv2.inRange(hsv, lower, upper)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500:  # Minimum area threshold
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    detections.append({
                        'class': color_name,
                        'confidence': 0.8,  # Fixed confidence for color detection
                        'bbox': [x, y, w, h],
                        'center': [x + w//2, y + h//2]
                    })
        
        return detections
    
    def annotate_frame(self, frame, detections):
        """Annotate frame with detection results"""
        annotated = frame.copy()
        
        for detection in detections:
            x, y, w, h = detection['bbox']
            class_name = detection['class']
            confidence = detection['confidence']
            
            # Draw bounding box
            color = self.colors[hash(class_name) % len(self.colors)]
            cv2.rectangle(annotated, (x, y), (x + w, y + h), color, 2)
            
            # Draw label
            label = f"{class_name}: {confidence:.2f}"
            cv2.putText(annotated, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Draw center point
            center = detection['center']
            cv2.circle(annotated, tuple(center), 5, color, -1)
        
        return annotated
    
    def publish_detections(self, detections):
        """Publish detection results"""
        detection_msg = String()
        detection_msg.data = json.dumps(detections)
        self.detection_pub.publish(detection_msg)
    
    def publish_annotated_image(self, frame):
        """Publish annotated image"""
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.image_pub.publish(img_msg)
        except Exception as e:
            rospy.logerr(f"Image publishing error: {e}")
    
    def pick_object_callback(self, msg):
        """Handle pick object command"""
        object_class = msg.data
        rospy.loginfo(f"🎯 Picking object: {object_class}")
        
        self.target_object = object_class
        self.servoing_active = True
        
        # Start visual servoing thread
        servo_thread = threading.Thread(target=self.pick_and_place_sequence)
        servo_thread.daemon = True
        servo_thread.start()
    
    def visual_servo(self, detections):
        """Visual servoing to target object"""
        target_detection = None
        
        for detection in detections:
            if detection['class'] == self.target_object:
                target_detection = detection
                break
        
        if target_detection is None:
            rospy.logwarn(f"Target object '{self.target_object}' not found")
            return
        
        # Calculate error from image center
        image_center = [320, 240]  # Assuming 640x480 image
        object_center = target_detection['center']
        
        error_x = object_center[0] - image_center[0]
        error_y = object_center[1] - image_center[1]
        
        # Simple proportional control
        kp = 0.001  # Proportional gain
        
        # Adjust base joint based on x error
        base_adjustment = -kp * error_x
        self.move_joint("Joint01", base_adjustment)
        
        # Adjust shoulder based on y error
        shoulder_adjustment = kp * error_y
        self.move_joint("Joint02", shoulder_adjustment)
        
        # Publish target point
        target_point = PointStamped()
        target_point.header.stamp = rospy.Time.now()
        target_point.header.frame_id = "camera_frame"
        target_point.point.x = object_center[0]
        target_point.point.y = object_center[1]
        target_point.point.z = 0
        
        self.target_pub.publish(target_point)
    
    def move_joint(self, joint_name, delta):
        """Move joint by delta amount"""
        if joint_name in self.current_positions:
            new_pos = self.current_positions[joint_name] + delta
            
            # Apply limits
            limits = {
                "Joint01": (-1.2, 1.2), "Joint02": (-1.0, 1.0), "Joint03": (0.0, 2.4),
                "Joint04": (-1.2, 1.2), "Joint05": (-1.0, 1.0), "Joint06": (-1.2, 1.2),
                "Gripper": (0.0, 0.6)
            }
            
            if joint_name in limits:
                min_pos, max_pos = limits[joint_name]
                new_pos = max(min_pos, min(max_pos, new_pos))
            
            self.current_positions[joint_name] = new_pos
            
            # Send command
            msg = MotorCmd()
            msg.mode = 10
            msg.q = float(new_pos)
            msg.Kp = 35.0
            msg.Kd = 1.5
            
            if joint_name == "Gripper":
                self.gripper_pub.publish(msg)
            else:
                self.joint_pubs[joint_name].publish(msg)
    
    def pick_and_place_sequence(self):
        """Automated pick and place sequence"""
        rospy.loginfo("🤖 Starting pick and place sequence")
        
        # Phase 1: Visual servoing to center object
        rospy.loginfo("📍 Phase 1: Centering object")
        start_time = time.time()
        
        while time.time() - start_time < 10.0:  # 10 second timeout
            if not self.servoing_active:
                break
            time.sleep(0.1)
        
        # Phase 2: Approach object
        rospy.loginfo("🎯 Phase 2: Approaching object")
        self.move_joint("Joint03", 0.5)  # Extend arm
        time.sleep(2)
        
        # Phase 3: Grasp object
        rospy.loginfo("🤏 Phase 3: Grasping object")
        self.move_joint("Gripper", 0.0)  # Close gripper
        time.sleep(1)
        
        # Phase 4: Lift object
        rospy.loginfo("⬆️ Phase 4: Lifting object")
        self.move_joint("Joint02", -0.3)  # Lift
        time.sleep(2)
        
        # Phase 5: Move to drop location
        rospy.loginfo("📦 Phase 5: Moving to drop location")
        self.move_joint("Joint01", 0.5)  # Rotate base
        time.sleep(2)
        
        # Phase 6: Release object
        rospy.loginfo("🔓 Phase 6: Releasing object")
        self.move_joint("Gripper", 0.6)  # Open gripper
        time.sleep(1)
        
        # Phase 7: Return home
        rospy.loginfo("🏠 Phase 7: Returning home")
        for joint in self.current_positions.keys():
            self.current_positions[joint] = 0.0
            self.move_joint(joint, 0.0)
        
        self.servoing_active = False
        self.target_object = None
        
        rospy.loginfo("✅ Pick and place sequence complete")
    
    def run(self):
        """Main execution loop"""
        rospy.loginfo("🎥 Z1 Computer Vision System ready")
        rospy.loginfo("📹 Camera feed processing active")
        rospy.loginfo("🎯 Send object class to /z1_vision/pick_object to start picking")
        
        # Keep node alive
        rospy.spin()

if __name__ == "__main__":
    try:
        cv_system = Z1ComputerVision()
        cv_system.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"❌ Computer vision error: {e}")
        rospy.logerr("💡 Install dependencies: pip3 install opencv-python")