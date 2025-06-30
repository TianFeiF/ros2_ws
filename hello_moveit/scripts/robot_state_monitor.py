#!/usr/bin/env python3
"""
Real-time Robot State Monitor
Monitor joint states, end-effector position and planned trajectory with real-time plotting
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading
import time
from datetime import datetime

class RobotStateMonitor(Node):
    def __init__(self):
        super().__init__('robot_state_monitor')
        
        # Data storage (max 1000 data points, about 100 seconds)
        self.max_points = 1000
        self.time_stamps = deque(maxlen=self.max_points)
        
        # Joint data
        self.joint_positions = {f'joint{i}': deque(maxlen=self.max_points) for i in range(1, 7)}
        self.joint_velocities = {f'joint{i}': deque(maxlen=self.max_points) for i in range(1, 7)}
        
        # End-effector actual position data
        self.end_effector_pos = {
            'x': deque(maxlen=self.max_points),
            'y': deque(maxlen=self.max_points),
            'z': deque(maxlen=self.max_points)
        }
        
        # End-effector actual orientation data (Euler angles)
        self.end_effector_rot = {
            'roll': deque(maxlen=self.max_points),
            'pitch': deque(maxlen=self.max_points),
            'yaw': deque(maxlen=self.max_points)
        }
        
        # Planned end-effector position data
        self.planned_end_effector_pos = {
            'x': deque(maxlen=self.max_points),
            'y': deque(maxlen=self.max_points),
            'z': deque(maxlen=self.max_points)
        }
        
        # Planned end-effector velocity data
        self.planned_end_effector_vel = {
            'x': deque(maxlen=self.max_points),
            'y': deque(maxlen=self.max_points),
            'z': deque(maxlen=self.max_points)
        }
        
        # Position difference data (actual - planned)
        self.position_error = {
            'x': deque(maxlen=self.max_points),
            'y': deque(maxlen=self.max_points),
            'z': deque(maxlen=self.max_points),
            'magnitude': deque(maxlen=self.max_points)
        }
        
        # Data lock
        self.data_lock = threading.Lock()
        
        # Subscribe to joint states
        self.joint_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Subscribe to planned trajectory
        self.trajectory_subscription = self.create_subscription(
            DisplayTrajectory,
            '/move_group/display_planned_path',
            self.trajectory_callback,
            10
        )
        
        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Start TF monitoring thread
        self.tf_thread = threading.Thread(target=self.tf_monitor_loop, daemon=True)
        self.tf_thread.start()
        
        self.get_logger().info('Robot State Monitor started')
        self.get_logger().info('Monitoring joint states, end-effector position and planned trajectory...')
        
    def joint_state_callback(self, msg):
        """Process joint state messages"""
        current_time = time.time()
        
        with self.data_lock:
            self.time_stamps.append(current_time)
            
            # Process joint positions
            for i, joint_name in enumerate(['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']):
                if i < len(msg.position):
                    self.joint_positions[joint_name].append(msg.position[i])
                else:
                    self.joint_positions[joint_name].append(0.0)
            
            # Process joint velocities
            for i, joint_name in enumerate(['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']):
                if i < len(msg.velocity):
                    velocity = msg.velocity[i]
                    self.joint_velocities[joint_name].append(velocity)
                else:
                    self.joint_velocities[joint_name].append(0.0)
    
    def trajectory_callback(self, msg):
        """Process planned trajectory messages"""
        if not msg.trajectory or len(msg.trajectory) == 0:
            return
            
        # Get the robot trajectory
        robot_trajectory = msg.trajectory[0]
        if not robot_trajectory.joint_trajectory.points:
            return
        
        # Get the last point of the trajectory (target point)
        last_point = robot_trajectory.joint_trajectory.points[-1]
        
        # Calculate end-effector position from joint positions using forward kinematics
        # This is a simplified calculation - in practice you'd use MoveIt's FK service
        planned_pos, planned_vel = self.calculate_end_effector_from_joints(
            last_point.positions, 
            last_point.velocities if last_point.velocities else [0.0]*6
        )
        
        current_time = time.time()
        
        with self.data_lock:
            # Store planned position
            self.planned_end_effector_pos['x'].append(planned_pos[0])
            self.planned_end_effector_pos['y'].append(planned_pos[1])
            self.planned_end_effector_pos['z'].append(planned_pos[2])
            
            # Store planned velocity
            self.planned_end_effector_vel['x'].append(planned_vel[0])
            self.planned_end_effector_vel['y'].append(planned_vel[1])
            self.planned_end_effector_vel['z'].append(planned_vel[2])
            
            # Calculate position error if we have actual position
            if (len(self.end_effector_pos['x']) > 0 and 
                len(self.end_effector_pos['y']) > 0 and 
                len(self.end_effector_pos['z']) > 0):
                
                actual_x = self.end_effector_pos['x'][-1]
                actual_y = self.end_effector_pos['y'][-1]
                actual_z = self.end_effector_pos['z'][-1]
                
                error_x = actual_x - planned_pos[0]
                error_y = actual_y - planned_pos[1]
                error_z = actual_z - planned_pos[2]
                error_mag = np.sqrt(error_x**2 + error_y**2 + error_z**2)
                
                self.position_error['x'].append(error_x)
                self.position_error['y'].append(error_y)
                self.position_error['z'].append(error_z)
                self.position_error['magnitude'].append(error_mag)
    
    def calculate_end_effector_from_joints(self, joint_positions, joint_velocities):
        """
        Simplified forward kinematics calculation
        In practice, you should use MoveIt's FK service for accurate results
        """
        # This is a placeholder - implement actual FK or use MoveIt FK service
        # For now, return dummy values
        planned_pos = [0.0, 0.0, 0.0]  # [x, y, z]
        planned_vel = [0.0, 0.0, 0.0]  # [vx, vy, vz]
        
        # You can implement simplified FK here or call MoveIt's FK service
        # For demonstration, we'll use a very basic approximation
        if len(joint_positions) >= 6:
            # Very rough approximation - replace with proper FK
            planned_pos[0] = 0.5 * np.cos(joint_positions[0]) * np.cos(joint_positions[1])
            planned_pos[1] = 0.5 * np.sin(joint_positions[0]) * np.cos(joint_positions[1])
            planned_pos[2] = 0.8 + 0.3 * np.sin(joint_positions[1])
            
            # Very rough velocity approximation
            if len(joint_velocities) >= 6:
                planned_vel[0] = -0.5 * np.sin(joint_positions[0]) * joint_velocities[0]
                planned_vel[1] = 0.5 * np.cos(joint_positions[0]) * joint_velocities[0]
                planned_vel[2] = 0.3 * np.cos(joint_positions[1]) * joint_velocities[1]
        
        return planned_pos, planned_vel
    
    def tf_monitor_loop(self):
        """TF monitoring loop"""
        rate = self.create_rate(10)  # 10Hz
        
        while rclpy.ok():
            try:
                # Get end-effector position
                transform = self.tf_buffer.lookup_transform(
                    'base_link', 'link6', 
                    rclpy.time.Time(), 
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                
                with self.data_lock:
                    # Position
                    self.end_effector_pos['x'].append(transform.transform.translation.x)
                    self.end_effector_pos['y'].append(transform.transform.translation.y)
                    self.end_effector_pos['z'].append(transform.transform.translation.z)
                    
                    # Convert quaternion to Euler angles
                    q = transform.transform.rotation
                    roll, pitch, yaw = self.quaternion_to_euler(q.x, q.y, q.z, q.w)
                    self.end_effector_rot['roll'].append(roll)
                    self.end_effector_rot['pitch'].append(pitch)
                    self.end_effector_rot['yaw'].append(yaw)
                    
            except Exception as e:
                self.get_logger().warn(f'TF lookup failed: {e}')
                
            time.sleep(0.1)  # 10Hz
    
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
    
    def get_plot_data(self):
        """Get plotting data"""
        with self.data_lock:
            if len(self.time_stamps) == 0:
                return None
                
            # Convert timestamps to relative time
            base_time = self.time_stamps[0] if self.time_stamps else 0
            times = [t - base_time for t in self.time_stamps]
            
            data = {
                'times': times,
                'joint_positions': {k: list(v) for k, v in self.joint_positions.items()},
                'joint_velocities': {k: list(v) for k, v in self.joint_velocities.items()},
                'end_effector_pos': {k: list(v) for k, v in self.end_effector_pos.items()},
                'end_effector_rot': {k: list(v) for k, v in self.end_effector_rot.items()},
                'planned_end_effector_pos': {k: list(v) for k, v in self.planned_end_effector_pos.items()},
                'planned_end_effector_vel': {k: list(v) for k, v in self.planned_end_effector_vel.items()},
                'position_error': {k: list(v) for k, v in self.position_error.items()}
            }
            
            return data

class RealTimePlotter:
    def __init__(self, monitor):
        self.monitor = monitor
        
        # Create figure window
        self.fig, self.axes = plt.subplots(3, 2, figsize=(16, 12))
        self.fig.suptitle('Real-time Robot State Monitor - Actual vs Planned', fontsize=16)
        
        # Set subplot titles
        titles = [
            'Joint Positions (rad)', 'End-Effector Position: Actual vs Planned (m)',
            'Joint Velocities (rad/s)', 'Planned End-Effector Velocity (m/s)', 
            'Position Error (m)', '3D Trajectory: Actual vs Planned'
        ]
        
        for i, ax in enumerate(self.axes.flat):
            ax.set_title(titles[i])
            ax.grid(True)
            ax.set_xlabel('Time (s)')
        
        # Initialize lines
        self.lines = {}
        
        # Joint position lines
        colors = ['red', 'green', 'blue', 'orange', 'purple', 'brown']
        for i, joint in enumerate(['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']):
            line, = self.axes[0, 0].plot([], [], color=colors[i], label=joint, linewidth=1)
            self.lines[f'pos_{joint}'] = line
        self.axes[0, 0].legend()
        
        # End-effector position lines (actual vs planned)
        for i, axis in enumerate(['x', 'y', 'z']):
            # Actual position (solid lines)
            line, = self.axes[0, 1].plot([], [], color=colors[i], label=f'actual_{axis}', linewidth=2)
            self.lines[f'end_pos_{axis}'] = line
            # Planned position (dashed lines)
            line, = self.axes[0, 1].plot([], [], color=colors[i], linestyle='--', label=f'planned_{axis}', linewidth=2)
            self.lines[f'planned_pos_{axis}'] = line
        self.axes[0, 1].legend()
        
        # Joint velocity lines
        for i, joint in enumerate(['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']):
            line, = self.axes[1, 0].plot([], [], color=colors[i], label=joint, linewidth=1)
            self.lines[f'vel_{joint}'] = line
        self.axes[1, 0].legend()
        
        # Planned end-effector velocity lines
        for i, axis in enumerate(['x', 'y', 'z']):
            line, = self.axes[1, 1].plot([], [], color=colors[i], label=f'vel_{axis}', linewidth=2)
            self.lines[f'planned_vel_{axis}'] = line
        self.axes[1, 1].legend()
        
        # Position error lines
        for i, axis in enumerate(['x', 'y', 'z']):
            line, = self.axes[2, 0].plot([], [], color=colors[i], label=f'error_{axis}', linewidth=2)
            self.lines[f'error_{axis}'] = line
        # Error magnitude
        line, = self.axes[2, 0].plot([], [], color='black', label='magnitude', linewidth=2)
        self.lines['error_magnitude'] = line
        self.axes[2, 0].legend()
        
        # 3D trajectory
        self.axes[2, 1].remove()
        self.ax_3d = self.fig.add_subplot(3, 2, 6, projection='3d')
        self.ax_3d.set_title('End-Effector 3D Trajectory: Actual vs Planned')
        self.ax_3d.set_xlabel('X (m)')
        self.ax_3d.set_ylabel('Y (m)')
        self.ax_3d.set_zlabel('Z (m)')
        # Actual trajectory (blue solid)
        self.actual_trajectory_line, = self.ax_3d.plot([], [], [], 'b-', linewidth=2, label='Actual')
        self.actual_current_point, = self.ax_3d.plot([], [], [], 'bo', markersize=8)
        # Planned trajectory (red dashed)
        self.planned_trajectory_line, = self.ax_3d.plot([], [], [], 'r--', linewidth=2, label='Planned')
        self.planned_current_point, = self.ax_3d.plot([], [], [], 'ro', markersize=8)
        self.ax_3d.legend()
        
        plt.tight_layout()
        
    def update_plots(self, frame):
        """Update plots"""
        data = self.monitor.get_plot_data()
        if data is None or len(data['times']) == 0:
            return list(self.lines.values())
        
        times = data['times']
        
        # Update joint positions
        for joint in ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']:
            if joint in data['joint_positions'] and len(data['joint_positions'][joint]) > 0:
                joint_data = data['joint_positions'][joint]
                plot_times = times[-len(joint_data):]
                self.lines[f'pos_{joint}'].set_data(plot_times, joint_data)
        
        # Update actual end-effector position
        for axis in ['x', 'y', 'z']:
            if axis in data['end_effector_pos'] and len(data['end_effector_pos'][axis]) > 0:
                pos_data = data['end_effector_pos'][axis]
                plot_times = times[-len(pos_data):]
                self.lines[f'end_pos_{axis}'].set_data(plot_times, pos_data)
        
        # Update planned end-effector position
        for axis in ['x', 'y', 'z']:
            if axis in data['planned_end_effector_pos'] and len(data['planned_end_effector_pos'][axis]) > 0:
                planned_pos_data = data['planned_end_effector_pos'][axis]
                plot_times = times[-len(planned_pos_data):]
                self.lines[f'planned_pos_{axis}'].set_data(plot_times, planned_pos_data)
        
        # Update joint velocities
        for joint in ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']:
            if joint in data['joint_velocities'] and len(data['joint_velocities'][joint]) > 0:
                vel_data = data['joint_velocities'][joint]
                plot_times = times[-len(vel_data):]
                self.lines[f'vel_{joint}'].set_data(plot_times, vel_data)
        
        # Update planned end-effector velocity
        for axis in ['x', 'y', 'z']:
            if axis in data['planned_end_effector_vel'] and len(data['planned_end_effector_vel'][axis]) > 0:
                planned_vel_data = data['planned_end_effector_vel'][axis]
                plot_times = times[-len(planned_vel_data):]
                self.lines[f'planned_vel_{axis}'].set_data(plot_times, planned_vel_data)
        
        # Update position error
        for axis in ['x', 'y', 'z']:
            if axis in data['position_error'] and len(data['position_error'][axis]) > 0:
                error_data = data['position_error'][axis]
                plot_times = times[-len(error_data):]
                self.lines[f'error_{axis}'].set_data(plot_times, error_data)
        
        # Update error magnitude
        if 'magnitude' in data['position_error'] and len(data['position_error']['magnitude']) > 0:
            error_mag_data = data['position_error']['magnitude']
            plot_times = times[-len(error_mag_data):]
            self.lines['error_magnitude'].set_data(plot_times, error_mag_data)
        
        # Update 3D trajectory - Actual
        if (len(data['end_effector_pos']['x']) > 0 and 
            len(data['end_effector_pos']['y']) > 0 and 
            len(data['end_effector_pos']['z']) > 0):
            
            x_data = data['end_effector_pos']['x']
            y_data = data['end_effector_pos']['y']
            z_data = data['end_effector_pos']['z']
            
            self.actual_trajectory_line.set_data_3d(x_data, y_data, z_data)
            
            if len(x_data) > 0:
                self.actual_current_point.set_data_3d([x_data[-1]], [y_data[-1]], [z_data[-1]])
        
        # Update 3D trajectory - Planned
        if (len(data['planned_end_effector_pos']['x']) > 0 and 
            len(data['planned_end_effector_pos']['y']) > 0 and 
            len(data['planned_end_effector_pos']['z']) > 0):
            
            x_planned = data['planned_end_effector_pos']['x']
            y_planned = data['planned_end_effector_pos']['y']
            z_planned = data['planned_end_effector_pos']['z']
            
            self.planned_trajectory_line.set_data_3d(x_planned, y_planned, z_planned)
            
            if len(x_planned) > 0:
                self.planned_current_point.set_data_3d([x_planned[-1]], [y_planned[-1]], [z_planned[-1]])
        
        # Auto-adjust axis ranges
        for ax in self.axes.flat:
            try:
                ax.relim()
                ax.autoscale_view()
            except:
                pass
                
        # Adjust 3D axis range
        try:
            all_x = []
            all_y = []
            all_z = []
            
            if len(data['end_effector_pos']['x']) > 0:
                all_x.extend(data['end_effector_pos']['x'])
                all_y.extend(data['end_effector_pos']['y'])
                all_z.extend(data['end_effector_pos']['z'])
            
            if len(data['planned_end_effector_pos']['x']) > 0:
                all_x.extend(data['planned_end_effector_pos']['x'])
                all_y.extend(data['planned_end_effector_pos']['y'])
                all_z.extend(data['planned_end_effector_pos']['z'])
            
            if len(all_x) > 0:
                x_range = [min(all_x) - 0.1, max(all_x) + 0.1]
                y_range = [min(all_y) - 0.1, max(all_y) + 0.1]
                z_range = [min(all_z) - 0.1, max(all_z) + 0.1]
                
                self.ax_3d.set_xlim(x_range)
                self.ax_3d.set_ylim(y_range)
                self.ax_3d.set_zlim(z_range)
        except:
            pass
        
        return list(self.lines.values())

def main():
    rclpy.init()
    
    # Create monitor
    monitor = RobotStateMonitor()
    
    # Run ROS in separate thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(monitor), daemon=True)
    ros_thread.start()
    
    # Wait for some data
    print("Starting monitor, waiting for data...")
    time.sleep(2)
    
    # Create plotter
    plotter = RealTimePlotter(monitor)
    
    # Start real-time plotting
    print("Starting real-time plotting...")
    ani = FuncAnimation(plotter.fig, plotter.update_plots, interval=100, blit=False)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        print("Program interrupted by user")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 