#!/usr/bin/env python3
"""
Robot Data Logger
Record joint states and end-effector position data to CSV files
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tf2_ros
import numpy as np
import csv
import time
from datetime import datetime
import os

class RobotDataLogger(Node):
    def __init__(self, output_dir="robot_data"):
        super().__init__('robot_data_logger')
        
        # Create output directory
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        
        # Generate filenames
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.joint_file = os.path.join(output_dir, f"joint_data_{timestamp}.csv")
        self.end_effector_file = os.path.join(output_dir, f"end_effector_data_{timestamp}.csv")
        
        # Initialize CSV files
        self.init_csv_files()
        
        # Subscribe to joint states
        self.joint_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Data counters
        self.joint_count = 0
        self.end_effector_count = 0
        
        # Start end-effector position recording timer (10Hz)
        self.end_effector_timer = self.create_timer(0.1, self.record_end_effector_pose)
        
        self.get_logger().info(f'Data Logger started')
        self.get_logger().info(f'Joint data file: {self.joint_file}')
        self.get_logger().info(f'End-effector data file: {self.end_effector_file}')
        
    def init_csv_files(self):
        """Initialize CSV file headers"""
        # Joint data file header
        with open(self.joint_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'time_sec',
                'joint1_pos', 'joint2_pos', 'joint3_pos', 'joint4_pos', 'joint5_pos', 'joint6_pos',
                'joint1_vel', 'joint2_vel', 'joint3_vel', 'joint4_vel', 'joint5_vel', 'joint6_vel',
                'joint1_eff', 'joint2_eff', 'joint3_eff', 'joint4_eff', 'joint5_eff', 'joint6_eff'
            ])
        
        # End-effector position file header
        with open(self.end_effector_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'time_sec',
                'pos_x', 'pos_y', 'pos_z',
                'quat_x', 'quat_y', 'quat_z', 'quat_w',
                'roll', 'pitch', 'yaw'
            ])
    
    def joint_state_callback(self, msg):
        """Record joint state data"""
        current_time = time.time()
        timestamp = datetime.now().isoformat()
        
        # Ensure 6 joints data
        positions = list(msg.position) + [0.0] * (6 - len(msg.position))
        velocities = list(msg.velocity) + [0.0] * (6 - len(msg.velocity))
        efforts = list(msg.effort) + [0.0] * (6 - len(msg.effort))
        
        # Take only first 6 joints
        positions = positions[:6]
        velocities = velocities[:6]
        efforts = efforts[:6]
        
        # Write to CSV
        with open(self.joint_file, 'a', newline='') as f:
            writer = csv.writer(f)
            row = [timestamp, current_time] + positions + velocities + efforts
            writer.writerow(row)
        
        self.joint_count += 1
        if self.joint_count % 100 == 0:
            self.get_logger().info(f'Recorded {self.joint_count} joint data entries')
    
    def record_end_effector_pose(self):
        """Record end-effector position data"""
        try:
            # Get end-effector position
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'link6', 
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            current_time = time.time()
            timestamp = datetime.now().isoformat()
            
            # Position
            pos_x = transform.transform.translation.x
            pos_y = transform.transform.translation.y
            pos_z = transform.transform.translation.z
            
            # Quaternion
            quat_x = transform.transform.rotation.x
            quat_y = transform.transform.rotation.y
            quat_z = transform.transform.rotation.z
            quat_w = transform.transform.rotation.w
            
            # Convert to Euler angles
            roll, pitch, yaw = self.quaternion_to_euler(quat_x, quat_y, quat_z, quat_w)
            
            # Write to CSV
            with open(self.end_effector_file, 'a', newline='') as f:
                writer = csv.writer(f)
                row = [timestamp, current_time, pos_x, pos_y, pos_z, 
                       quat_x, quat_y, quat_z, quat_w, roll, pitch, yaw]
                writer.writerow(row)
            
            self.end_effector_count += 1
            if self.end_effector_count % 100 == 0:
                self.get_logger().info(f'Recorded {self.end_effector_count} end-effector data entries')
                
        except Exception as e:
            self.get_logger().warn(f'End-effector recording failed: {e}')
    
    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw

def main():
    rclpy.init()
    
    logger = RobotDataLogger()
    
    try:
        print("Starting data recording, press Ctrl+C to stop...")
        rclpy.spin(logger)
    except KeyboardInterrupt:
        print(f"\nData recording completed!")
        print(f"Joint data entries: {logger.joint_count}")
        print(f"End-effector data entries: {logger.end_effector_count}")
        print(f"Files saved in: {logger.output_dir}/")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 