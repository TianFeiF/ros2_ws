#!/usr/bin/env python3
"""
Robot Data Analyzer
Read recorded CSV data and generate analysis charts
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import glob
from datetime import datetime
import argparse

class RobotDataAnalyzer:
    def __init__(self, data_dir="robot_data"):
        self.data_dir = data_dir
        self.joint_data = None
        self.end_effector_data = None
        
    def load_latest_data(self):
        """Load latest data files"""
        # Find latest data files
        joint_files = glob.glob(os.path.join(self.data_dir, "joint_data_*.csv"))
        end_effector_files = glob.glob(os.path.join(self.data_dir, "end_effector_data_*.csv"))
        
        if not joint_files or not end_effector_files:
            print("No data files found!")
            return False
        
        # Select latest files
        latest_joint_file = max(joint_files, key=os.path.getctime)
        latest_end_effector_file = max(end_effector_files, key=os.path.getctime)
        
        print(f"Loading joint data: {latest_joint_file}")
        print(f"Loading end-effector data: {latest_end_effector_file}")
        
        try:
            self.joint_data = pd.read_csv(latest_joint_file)
            self.end_effector_data = pd.read_csv(latest_end_effector_file)
            
            # Convert timestamps
            self.joint_data['time_rel'] = self.joint_data['time_sec'] - self.joint_data['time_sec'].iloc[0]
            self.end_effector_data['time_rel'] = self.end_effector_data['time_sec'] - self.end_effector_data['time_sec'].iloc[0]
            
            print(f"Joint data: {len(self.joint_data)} records")
            print(f"End-effector data: {len(self.end_effector_data)} records")
            return True
            
        except Exception as e:
            print(f"Data loading failed: {e}")
            return False
    
    def load_specific_data(self, joint_file, end_effector_file):
        """Load specific data files"""
        try:
            self.joint_data = pd.read_csv(joint_file)
            self.end_effector_data = pd.read_csv(end_effector_file)
            
            # Convert timestamps
            self.joint_data['time_rel'] = self.joint_data['time_sec'] - self.joint_data['time_sec'].iloc[0]
            self.end_effector_data['time_rel'] = self.end_effector_data['time_sec'] - self.end_effector_data['time_sec'].iloc[0]
            
            return True
        except Exception as e:
            print(f"Data loading failed: {e}")
            return False
    
    def plot_joint_analysis(self):
        """Plot joint analysis charts"""
        if self.joint_data is None:
            print("Please load data first!")
            return
        
        fig, axes = plt.subplots(3, 2, figsize=(15, 12))
        fig.suptitle('Joint Motion Analysis', fontsize=16)
        
        # Joint positions
        ax = axes[0, 0]
        for i in range(1, 7):
            ax.plot(self.joint_data['time_rel'], self.joint_data[f'joint{i}_pos'], 
                   label=f'Joint {i}', linewidth=1)
        ax.set_title('Joint Positions')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (rad)')
        ax.legend()
        ax.grid(True)
        
        # Joint velocities
        ax = axes[0, 1]
        for i in range(1, 7):
            ax.plot(self.joint_data['time_rel'], self.joint_data[f'joint{i}_vel'], 
                   label=f'Joint {i}', linewidth=1)
        ax.set_title('Joint Velocities')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (rad/s)')
        ax.legend()
        ax.grid(True)
        
        # Joint efforts
        ax = axes[1, 0]
        for i in range(1, 7):
            ax.plot(self.joint_data['time_rel'], self.joint_data[f'joint{i}_eff'], 
                   label=f'Joint {i}', linewidth=1)
        ax.set_title('Joint Efforts')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Effort (Nm)')
        ax.legend()
        ax.grid(True)
        
        # Velocity distribution histogram
        ax = axes[1, 1]
        for i in range(1, 7):
            ax.hist(self.joint_data[f'joint{i}_vel'], bins=30, alpha=0.6, label=f'Joint {i}')
        ax.set_title('Joint Velocity Distribution')
        ax.set_xlabel('Velocity (rad/s)')
        ax.set_ylabel('Frequency')
        ax.legend()
        ax.grid(True)
        
        # Position range analysis
        ax = axes[2, 0]
        joint_names = [f'Joint {i}' for i in range(1, 7)]
        pos_ranges = []
        pos_means = []
        for i in range(1, 7):
            pos_data = self.joint_data[f'joint{i}_pos']
            pos_ranges.append(pos_data.max() - pos_data.min())
            pos_means.append(pos_data.mean())
        
        x = np.arange(len(joint_names))
        ax.bar(x, pos_ranges, alpha=0.7)
        ax.set_title('Joint Position Range')
        ax.set_xlabel('Joint')
        ax.set_ylabel('Position Range (rad)')
        ax.set_xticks(x)
        ax.set_xticklabels(joint_names)
        ax.grid(True)
        
        # Velocity statistics
        ax = axes[2, 1]
        vel_max = []
        vel_rms = []
        for i in range(1, 7):
            vel_data = self.joint_data[f'joint{i}_vel']
            vel_max.append(abs(vel_data).max())
            vel_rms.append(np.sqrt(np.mean(vel_data**2)))
        
        x = np.arange(len(joint_names))
        width = 0.35
        ax.bar(x - width/2, vel_max, width, label='Max Velocity', alpha=0.7)
        ax.bar(x + width/2, vel_rms, width, label='RMS Velocity', alpha=0.7)
        ax.set_title('Joint Velocity Statistics')
        ax.set_xlabel('Joint')
        ax.set_ylabel('Velocity (rad/s)')
        ax.set_xticks(x)
        ax.set_xticklabels(joint_names)
        ax.legend()
        ax.grid(True)
        
        plt.tight_layout()
        plt.show()
    
    def plot_end_effector_analysis(self):
        """Plot end-effector analysis charts"""
        if self.end_effector_data is None:
            print("Please load data first!")
            return
        
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('End-Effector Motion Analysis', fontsize=16)
        
        # Position trajectory
        ax = axes[0, 0]
        ax.plot(self.end_effector_data['time_rel'], self.end_effector_data['pos_x'], 'r-', label='X', linewidth=2)
        ax.plot(self.end_effector_data['time_rel'], self.end_effector_data['pos_y'], 'g-', label='Y', linewidth=2)
        ax.plot(self.end_effector_data['time_rel'], self.end_effector_data['pos_z'], 'b-', label='Z', linewidth=2)
        ax.set_title('End-Effector Position Trajectory')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (m)')
        ax.legend()
        ax.grid(True)
        
        # Orientation trajectory
        ax = axes[0, 1]
        ax.plot(self.end_effector_data['time_rel'], np.degrees(self.end_effector_data['roll']), 'r-', label='Roll', linewidth=2)
        ax.plot(self.end_effector_data['time_rel'], np.degrees(self.end_effector_data['pitch']), 'g-', label='Pitch', linewidth=2)
        ax.plot(self.end_effector_data['time_rel'], np.degrees(self.end_effector_data['yaw']), 'b-', label='Yaw', linewidth=2)
        ax.set_title('End-Effector Orientation Trajectory')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (degrees)')
        ax.legend()
        ax.grid(True)
        
        # 3D trajectory
        ax = axes[1, 0]
        ax.remove()
        ax = fig.add_subplot(2, 2, 3, projection='3d')
        ax.plot(self.end_effector_data['pos_x'], 
                self.end_effector_data['pos_y'], 
                self.end_effector_data['pos_z'], 'b-', linewidth=2)
        ax.scatter(self.end_effector_data['pos_x'].iloc[0], 
                  self.end_effector_data['pos_y'].iloc[0], 
                  self.end_effector_data['pos_z'].iloc[0], 
                  color='green', s=100, label='Start')
        ax.scatter(self.end_effector_data['pos_x'].iloc[-1], 
                  self.end_effector_data['pos_y'].iloc[-1], 
                  self.end_effector_data['pos_z'].iloc[-1], 
                  color='red', s=100, label='End')
        ax.set_title('3D Trajectory')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.legend()
        
        # Position velocity
        ax = axes[1, 1]
        # Calculate position velocity
        dt = np.diff(self.end_effector_data['time_rel'])
        dx = np.diff(self.end_effector_data['pos_x'])
        dy = np.diff(self.end_effector_data['pos_y'])
        dz = np.diff(self.end_effector_data['pos_z'])
        
        # Avoid division by zero
        dt[dt == 0] = 1e-6
        
        vel_x = dx / dt
        vel_y = dy / dt
        vel_z = dz / dt
        vel_total = np.sqrt(vel_x**2 + vel_y**2 + vel_z**2)
        
        time_vel = self.end_effector_data['time_rel'].iloc[1:]
        ax.plot(time_vel, vel_x, 'r-', label='Vx', alpha=0.7)
        ax.plot(time_vel, vel_y, 'g-', label='Vy', alpha=0.7)
        ax.plot(time_vel, vel_z, 'b-', label='Vz', alpha=0.7)
        ax.plot(time_vel, vel_total, 'k-', label='Total Velocity', linewidth=2)
        ax.set_title('End-Effector Velocity')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (m/s)')
        ax.legend()
        ax.grid(True)
        
        plt.tight_layout()
        plt.show()
    
    def generate_report(self):
        """Generate data analysis report"""
        if self.joint_data is None or self.end_effector_data is None:
            print("Please load data first!")
            return
        
        print("\n" + "="*60)
        print("Robot Motion Data Analysis Report")
        print("="*60)
        
        # Time information
        duration = self.joint_data['time_rel'].iloc[-1]
        print(f"\nRecording duration: {duration:.2f} seconds")
        print(f"Joint data points: {len(self.joint_data)}")
        print(f"End-effector data points: {len(self.end_effector_data)}")
        
        # Joint analysis
        print(f"\nJoint Motion Analysis:")
        print("-" * 40)
        for i in range(1, 7):
            pos_data = self.joint_data[f'joint{i}_pos']
            vel_data = self.joint_data[f'joint{i}_vel']
            eff_data = self.joint_data[f'joint{i}_eff']
            
            print(f"Joint {i}:")
            print(f"  Position range: {pos_data.min():.3f} ~ {pos_data.max():.3f} rad")
            print(f"  Max velocity: {abs(vel_data).max():.3f} rad/s")
            print(f"  Average effort: {abs(eff_data).mean():.3f} Nm")
        
        # End-effector analysis
        print(f"\nEnd-Effector Analysis:")
        print("-" * 40)
        
        # Position range
        x_range = self.end_effector_data['pos_x'].max() - self.end_effector_data['pos_x'].min()
        y_range = self.end_effector_data['pos_y'].max() - self.end_effector_data['pos_y'].min()
        z_range = self.end_effector_data['pos_z'].max() - self.end_effector_data['pos_z'].min()
        
        print(f"Position range:")
        print(f"  X-axis: {x_range:.3f} m")
        print(f"  Y-axis: {y_range:.3f} m")
        print(f"  Z-axis: {z_range:.3f} m")
        
        # Orientation range
        roll_range = np.degrees(self.end_effector_data['roll'].max() - self.end_effector_data['roll'].min())
        pitch_range = np.degrees(self.end_effector_data['pitch'].max() - self.end_effector_data['pitch'].min())
        yaw_range = np.degrees(self.end_effector_data['yaw'].max() - self.end_effector_data['yaw'].min())
        
        print(f"Orientation range:")
        print(f"  Roll: {roll_range:.1f}°")
        print(f"  Pitch: {pitch_range:.1f}°")
        print(f"  Yaw: {yaw_range:.1f}°")
        
        # Motion distance
        dx = np.diff(self.end_effector_data['pos_x'])
        dy = np.diff(self.end_effector_data['pos_y'])
        dz = np.diff(self.end_effector_data['pos_z'])
        distances = np.sqrt(dx**2 + dy**2 + dz**2)
        total_distance = np.sum(distances)
        
        print(f"Total motion distance: {total_distance:.3f} m")
        print(f"Average motion speed: {total_distance/duration:.3f} m/s")
        
        print("\n" + "="*60)

def main():
    parser = argparse.ArgumentParser(description='Analyze robot motion data')
    parser.add_argument('--data-dir', default='robot_data', help='Data directory')
    parser.add_argument('--joint-file', help='Specify joint data file')
    parser.add_argument('--end-effector-file', help='Specify end-effector data file')
    parser.add_argument('--no-plot', action='store_true', help='Do not show plots')
    
    args = parser.parse_args()
    
    analyzer = RobotDataAnalyzer(args.data_dir)
    
    # Load data
    if args.joint_file and args.end_effector_file:
        success = analyzer.load_specific_data(args.joint_file, args.end_effector_file)
    else:
        success = analyzer.load_latest_data()
    
    if not success:
        return
    
    # Generate report
    analyzer.generate_report()
    
    # Show plots
    if not args.no_plot:
        print("\nShowing joint analysis charts...")
        analyzer.plot_joint_analysis()
        
        print("Showing end-effector analysis charts...")
        analyzer.plot_end_effector_analysis()

if __name__ == '__main__':
    main() 