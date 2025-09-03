#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Duration
import numpy as np

class Z1ControllerROS2(Node):
    def __init__(self):
        super().__init__('z1_controller_ros2')
        
        # Action clients for ROS2 control
        self.arm_action_client = ActionClient(
            self, FollowJointTrajectory, '/z1_arm_controller/follow_joint_trajectory'
        )
        
        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )
        self.cmd_sub = self.create_subscription(
            Twist, '/z1/cmd_vel', self.cmd_callback, 10
        )
        
        # Current joint state
        self.current_joint_positions = [0.0] * 6
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # Joint limits
        self.joint_limits = {
            'joint1': (-2.618, 2.618),
            'joint2': (0.0, 2.967),
            'joint3': (-2.88, 0.0),
            'joint4': (-1.518, 1.518),
            'joint5': (-1.344, 1.344),
            'joint6': (-2.793, 2.793)
        }
        
        self.get_logger().info('Z1 ROS2 Controller initialized')
        
        # Wait for action server
        self.arm_action_client.wait_for_server()
        self.get_logger().info('Connected to arm controller')
    
    def joint_callback(self, msg):
        """Update current joint positions"""
        if len(msg.position) >= 6:
            self.current_joint_positions = list(msg.position[:6])
    
    def cmd_callback(self, msg):
        """Handle velocity commands and convert to position targets"""
        # Calculate new target positions based on velocity commands
        target_positions = []
        velocity_scale = 0.1  # Scale factor for velocity to position conversion
        
        for i, (current_pos, joint_name) in enumerate(zip(self.current_joint_positions, self.joint_names)):
            # Map Twist components to joints
            if i == 0:  # Joint 1 - base rotation
                velocity = msg.angular.z
            elif i == 1:  # Joint 2 - shoulder
                velocity = msg.linear.y
            elif i == 2:  # Joint 3 - elbow
                velocity = msg.linear.z
            elif i == 3:  # Joint 4 - forearm
                velocity = msg.angular.x
            elif i == 4:  # Joint 5 - wrist pitch
                velocity = msg.angular.y
            elif i == 5:  # Joint 6 - wrist roll
                velocity = msg.linear.x
            else:
                velocity = 0.0
            
            # Calculate new position
            new_pos = current_pos + (velocity * velocity_scale)
            
            # Apply joint limits
            min_limit, max_limit = self.joint_limits[joint_name]
            new_pos = max(min_limit, min(max_limit, new_pos))
            
            target_positions.append(new_pos)
        
        # Send trajectory command
        self.send_joint_trajectory(target_positions, 0.1)  # 0.1 second duration
    
    def send_joint_trajectory(self, positions, duration_sec):
        """Send joint trajectory to ROS2 control"""
        goal_msg = FollowJointTrajectory.Goal()
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        
        trajectory.points = [point]
        goal_msg.trajectory = trajectory
        
        # Send goal
        self.arm_action_client.send_goal_async(goal_msg)
    
    def home_position(self):
        """Move to home position"""
        home_positions = [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]  # Safe home position
        self.send_joint_trajectory(home_positions, 2.0)  # 2 second duration
        self.get_logger().info('Moving to home position')
    
    def wave_gesture(self):
        """Perform wave gesture"""
        wave_positions = [
            [0.0, 0.5, -0.5, 0.0, 1.0, 0.0],   # Raise arm
            [0.0, 0.5, -0.5, 0.0, 1.0, 1.0],   # Wave right
            [0.0, 0.5, -0.5, 0.0, 1.0, -1.0],  # Wave left
            [0.0, 0.5, -0.5, 0.0, 1.0, 0.0],   # Center
            [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]    # Lower arm
        ]
        
        for i, positions in enumerate(wave_positions):
            self.send_joint_trajectory(positions, 1.0)
            self.get_logger().info(f'Wave step {i+1}/5')

def main(args=None):
    rclpy.init(args=args)
    controller = Z1ControllerROS2()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()