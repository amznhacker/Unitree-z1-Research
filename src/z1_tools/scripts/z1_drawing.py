#!/usr/bin/env python3
"""
Z1 Drawing Demo
Make the robotic arm draw simple shapes in 3D space
"""
import rospy, math, time, argparse
from unitree_legged_msgs.msg import MotorCmd

class Z1Drawing:
    def __init__(self, mode=10, kp=35.0, kd=1.5):
        rospy.init_node("z1_drawing")
        
        self.mode = mode
        self.kp = kp
        self.kd = kd
        
        # Publishers
        self.pubs = {}
        joints = ["Joint01", "Joint02", "Joint03", "Joint04", "Joint05", "Joint06"]
        for joint in joints:
            topic = f"/z1_gazebo/{joint}_controller/command"
            self.pubs[joint] = rospy.Publisher(topic, MotorCmd, queue_size=10)
        
        self.rate = rospy.Rate(50)
        
        # Drawing parameters
        self.drawing_height = 0.3  # Height above base for drawing
        self.drawing_radius = 0.2  # Radius for circular motions
    
    def send_joint_positions(self, positions):
        """Send positions to all joints"""
        joints = ["Joint01", "Joint02", "Joint03", "Joint04", "Joint05", "Joint06"]
        
        for i, joint in enumerate(joints):
            if i < len(positions):
                msg = MotorCmd()
                msg.mode = self.mode
                msg.q = float(positions[i])
                msg.dq = 0.0
                msg.tau = 0.0
                msg.Kp = self.kp
                msg.Kd = self.kd
                self.pubs[joint].publish(msg)
    
    def move_to_start_position(self):
        """Move to drawing start position"""
        rospy.loginfo("Moving to drawing start position")
        
        # Safe drawing pose: slightly forward, elbow bent
        start_pose = [0.0, -0.4, 1.0, 0.0, -0.6, 0.0]
        
        # Smooth movement to start
        duration = 3.0
        start_time = time.time()
        
        while not rospy.is_shutdown():
            elapsed = time.time() - start_time
            if elapsed >= duration:
                break
            
            alpha = elapsed / duration
            smooth_alpha = 0.5 * (1 - math.cos(math.pi * alpha))
            
            current_pose = [pos * smooth_alpha for pos in start_pose]
            self.send_joint_positions(current_pose)
            self.rate.sleep()
        
        # Hold position
        for _ in range(50):
            self.send_joint_positions(start_pose)
            self.rate.sleep()
    
    def draw_circle(self, radius=0.15, duration=10.0):
        """Draw a circle in vertical plane"""
        rospy.loginfo(f"Drawing circle with radius {radius}m over {duration}s")
        
        start_time = time.time()
        
        while not rospy.is_shutdown():
            elapsed = time.time() - start_time
            if elapsed >= duration:
                break
            
            # Circle parameter
            t = (elapsed / duration) * 2 * math.pi
            
            # Calculate circle positions
            # Use shoulder and elbow to create circular motion
            base_shoulder = -0.4
            base_elbow = 1.0
            
            # Circular motion in shoulder-elbow space
            shoulder_offset = radius * math.sin(t) * 0.5
            elbow_offset = radius * math.cos(t) * 0.3
            
            positions = [
                0.0,  # Base stays centered
                base_shoulder + shoulder_offset,
                base_elbow + elbow_offset,
                0.0,  # Forearm neutral
                -0.6 - shoulder_offset,  # Wrist compensates
                0.0   # Wrist roll neutral
            ]
            
            self.send_joint_positions(positions)
            self.rate.sleep()
    
    def draw_figure_eight(self, size=0.12, duration=15.0):
        """Draw a figure-8 pattern"""
        rospy.loginfo(f"Drawing figure-8 with size {size}m over {duration}s")
        
        start_time = time.time()
        
        while not rospy.is_shutdown():
            elapsed = time.time() - start_time
            if elapsed >= duration:
                break
            
            # Figure-8 parameter
            t = (elapsed / duration) * 4 * math.pi  # Two loops
            
            # Lissajous curve for figure-8
            x = size * math.sin(t)
            y = size * math.sin(2 * t) / 2
            
            # Map to joint space
            positions = [
                x * 2.0,  # Base rotation for X
                -0.4 + y * 1.5,  # Shoulder for Y
                1.0 - abs(x) * 0.5,  # Elbow varies with X
                0.0,
                -0.6 + y * 1.0,  # Wrist pitch for Y
                x * 1.5   # Wrist roll for X
            ]
            
            self.send_joint_positions(positions)
            self.rate.sleep()
    
    def draw_square(self, size=0.1, duration=12.0):
        """Draw a square pattern"""
        rospy.loginfo(f"Drawing square with size {size}m over {duration}s")
        
        # Square corners in joint space
        corners = [
            [0.0, -0.3, 0.8, 0.0, -0.5, 0.0],    # Top-left
            [0.3, -0.3, 0.8, 0.0, -0.5, 0.0],    # Top-right
            [0.3, -0.5, 1.2, 0.0, -0.7, 0.0],    # Bottom-right
            [0.0, -0.5, 1.2, 0.0, -0.7, 0.0],    # Bottom-left
            [0.0, -0.3, 0.8, 0.0, -0.5, 0.0]     # Back to start
        ]
        
        segment_duration = duration / len(corners)
        
        for i in range(len(corners) - 1):
            if rospy.is_shutdown():
                break
            
            start_pos = corners[i]
            end_pos = corners[i + 1]
            
            rospy.loginfo(f"Drawing line segment {i + 1}")
            
            # Linear interpolation between corners
            start_time = time.time()
            while not rospy.is_shutdown():
                elapsed = time.time() - start_time
                if elapsed >= segment_duration:
                    break
                
                alpha = elapsed / segment_duration
                
                current_pos = []
                for j in range(len(start_pos)):
                    pos = start_pos[j] + alpha * (end_pos[j] - start_pos[j])
                    current_pos.append(pos)
                
                self.send_joint_positions(current_pos)
                self.rate.sleep()
    
    def return_home(self):
        """Return to home position"""
        rospy.loginfo("Returning to home position")
        
        duration = 3.0
        start_time = time.time()
        
        while not rospy.is_shutdown():
            elapsed = time.time() - start_time
            if elapsed >= duration:
                break
            
            alpha = 1.0 - (elapsed / duration)
            smooth_alpha = 0.5 * (1 - math.cos(math.pi * (1 - alpha)))
            
            # Gradually return to neutral
            positions = [0.0 * smooth_alpha] * 6
            self.send_joint_positions(positions)
            self.rate.sleep()
        
        # Hold home position
        for _ in range(50):
            self.send_joint_positions([0.0] * 6)
            self.rate.sleep()

def main():
    parser = argparse.ArgumentParser(description="Z1 Drawing Demo")
    parser.add_argument("--mode", type=int, default=10, help="Controller mode")
    parser.add_argument("--kp", type=float, default=35.0, help="Proportional gain")
    parser.add_argument("--kd", type=float, default=1.5, help="Derivative gain")
    parser.add_argument("--shape", choices=["circle", "figure8", "square", "all"],
                       default="all", help="Shape to draw")
    parser.add_argument("--size", type=float, default=0.12, help="Drawing size")
    parser.add_argument("--speed", type=float, default=1.0, help="Speed multiplier")
    args = parser.parse_args()
    
    try:
        drawer = Z1Drawing(args.mode, args.kp, args.kd)
        
        # Move to start position
        drawer.move_to_start_position()
        time.sleep(1.0)
        
        # Draw requested shapes
        if args.shape == "circle" or args.shape == "all":
            drawer.draw_circle(args.size, 10.0 / args.speed)
            time.sleep(2.0)
        
        if args.shape == "figure8" or args.shape == "all":
            drawer.draw_figure_eight(args.size, 15.0 / args.speed)
            time.sleep(2.0)
        
        if args.shape == "square" or args.shape == "all":
            drawer.draw_square(args.size, 12.0 / args.speed)
            time.sleep(2.0)
        
        # Return home
        drawer.return_home()
        
        rospy.loginfo("Drawing demo completed")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Drawing demo interrupted")

if __name__ == "__main__":
    main()