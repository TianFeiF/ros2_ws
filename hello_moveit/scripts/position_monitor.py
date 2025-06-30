#!/usr/bin/env python3
"""
Position Monitor - 实际位置与规划位置对比监控
专门监控末端执行器的实际位置和规划位置，显示位置误差
"""

import rclpy
from rclcpp.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.srv import GetPositionFK
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading
import time

class PositionMonitor(Node):
    def __init__(self):
        super().__init__('position_monitor')
        
        # Data storage (max 500 data points)
        self.max_points = 500
        self.time_stamps = deque(maxlen=self.max_points)
        
        # 实际末端位置数据
        self.actual_pos = {
            'x': deque(maxlen=self.max_points),
            'y': deque(maxlen=self.max_points),
            'z': deque(maxlen=self.max_points)
        }
        
        # 规划末端位置数据
        self.planned_pos = {
            'x': deque(maxlen=self.max_points),
            'y': deque(maxlen=self.max_points),
            'z': deque(maxlen=self.max_points)
        }
        
        # 位置误差数据
        self.position_error = {
            'x': deque(maxlen=self.max_points),
            'y': deque(maxlen=self.max_points),
            'z': deque(maxlen=self.max_points),
            'magnitude': deque(maxlen=self.max_points)
        }
        
        # 当前关节状态
        self.current_joint_state = None
        
        # 数据锁
        self.data_lock = threading.Lock()
        
        # 订阅关节状态
        self.joint_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # 订阅规划轨迹
        self.trajectory_subscription = self.create_subscription(
            DisplayTrajectory,
            '/display_planned_path',
            self.trajectory_callback,
            10
        )
        
        # TF监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # FK服务客户端
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        
        # 启动TF监控线程
        self.tf_thread = threading.Thread(target=self.tf_monitor_loop, daemon=True)
        self.tf_thread.start()
        
        self.get_logger().info('位置监控器启动 - 监控实际位置与规划位置对比')
        
    def joint_state_callback(self, msg):
        """处理关节状态消息"""
        self.current_joint_state = msg
        
    def trajectory_callback(self, msg):
        """处理规划轨迹消息"""
        # 减少调试信息频率
        static_counter = getattr(self, '_trajectory_counter', 0) + 1
        self._trajectory_counter = static_counter
        
        if static_counter % 10 == 1:  # 每10次消息输出一次
            self.get_logger().info('收到规划轨迹消息')
        
        if not msg.trajectory or len(msg.trajectory) == 0:
            if static_counter % 10 == 1:
                self.get_logger().warn('轨迹消息为空')
            return
            
        # 获取机器人轨迹
        robot_trajectory = msg.trajectory[0]
        if not robot_trajectory.joint_trajectory.points:
            if static_counter % 10 == 1:
                self.get_logger().warn('关节轨迹点为空')
            return
        
        if static_counter % 10 == 1:
            self.get_logger().info(f'轨迹包含 {len(robot_trajectory.joint_trajectory.points)} 个点')
        
        # 获取轨迹的最后一点（目标点）
        last_point = robot_trajectory.joint_trajectory.points[-1]
        
        if static_counter % 10 == 1:
            self.get_logger().info(f'目标关节位置: {[f"{pos:.3f}" for pos in last_point.positions]}')
        
        # 使用FK服务计算末端位置
        self.calculate_planned_position(last_point.positions)
        
    def calculate_planned_position(self, joint_positions):
        """使用FK服务计算规划的末端位置"""
        if not self.fk_client.wait_for_service(timeout_sec=0.1):
            # FK服务不可用，使用简化计算
            self.calculate_planned_position_simple(joint_positions)
            return
            
        try:
            # 创建FK请求
            request = GetPositionFK.Request()
            request.header.frame_id = 'base_link'
            request.fk_link_names = ['link6']
            
            # 设置关节状态
            request.robot_state.joint_state.name = [
                'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
            ]
            request.robot_state.joint_state.position = list(joint_positions)
            
            # 异步调用FK服务
            future = self.fk_client.call_async(request)
            future.add_done_callback(self.fk_response_callback)
            
        except Exception as e:
            self.get_logger().warn(f'FK服务调用失败: {e}')
            self.calculate_planned_position_simple(joint_positions)
    
    def fk_response_callback(self, future):
        """处理FK服务响应"""
        try:
            response = future.result()
            if response.error_code.val == 1 and len(response.pose_stamped) > 0:
                pose = response.pose_stamped[0].pose
                current_time = time.time()
                
                with self.data_lock:
                    # 存储规划位置
                    self.planned_pos['x'].append(pose.position.x)
                    self.planned_pos['y'].append(pose.position.y)
                    self.planned_pos['z'].append(pose.position.z)
                    
                    # 计算位置误差
                    self.calculate_position_error(pose.position.x, pose.position.y, pose.position.z)
                    
        except Exception as e:
            self.get_logger().warn(f'FK响应处理失败: {e}')
    
    def calculate_planned_position_simple(self, joint_positions):
        """简化的正向运动学计算"""
        if len(joint_positions) < 6:
            return
            
        # 获取关节角度
        j1, j2, j3, j4, j5, j6 = joint_positions[:6]
        
        # 使用更接近真实机械臂的DH参数（这里使用通用6DOF机械臂的近似值）
        # 这些参数需要根据你的具体机械臂型号调整
        a1, a2, a3 = 0.0, 0.42, 0.39  # 连杆长度
        d1, d4, d6 = 0.15, 0.12, 0.10  # 连杆偏移
        
        # 简化的正向运动学计算
        # 这是一个近似计算，实际应该使用完整的DH变换矩阵
        
        # Base rotation
        cos1, sin1 = np.cos(j1), np.sin(j1)
        cos2, sin2 = np.cos(j2), np.sin(j2)
        cos23, sin23 = np.cos(j2 + j3), np.sin(j2 + j3)
        
        # 计算腕部位置（忽略j4, j5, j6对位置的影响，只考虑姿态）
        x_wrist = cos1 * (a2 * cos2 + a3 * cos23)
        y_wrist = sin1 * (a2 * cos2 + a3 * cos23)
        z_wrist = d1 + a2 * sin2 + a3 * sin23
        
        # 添加末端执行器偏移（简化处理）
        cos4, sin4 = np.cos(j4), np.sin(j4)
        cos5, sin5 = np.cos(j5), np.sin(j5)
        
        # 末端执行器位置（考虑j4, j5对位置的小影响）
        x = x_wrist + d6 * cos1 * cos23 * cos5
        y = y_wrist + d6 * sin1 * cos23 * cos5
        z = z_wrist + d6 * sin23 * cos5
        
        # 减少调试信息频率
        static_counter = getattr(self, '_fk_counter', 0) + 1
        self._fk_counter = static_counter
        
        if static_counter % 20 == 1:  # 每20次计算输出一次
            self.get_logger().info(f'计算的规划位置: x={x:.3f}, y={y:.3f}, z={z:.3f}')
        
        current_time = time.time()
        
        with self.data_lock:
            # 存储规划位置
            self.planned_pos['x'].append(x)
            self.planned_pos['y'].append(y)
            self.planned_pos['z'].append(z)
            
            # 计算位置误差
            self.calculate_position_error(x, y, z)
    
    def calculate_position_error(self, planned_x, planned_y, planned_z):
        """计算位置误差"""
        if (len(self.actual_pos['x']) > 0 and 
            len(self.actual_pos['y']) > 0 and 
            len(self.actual_pos['z']) > 0):
            
            actual_x = self.actual_pos['x'][-1]
            actual_y = self.actual_pos['y'][-1]
            actual_z = self.actual_pos['z'][-1]
            
            error_x = actual_x - planned_x
            error_y = actual_y - planned_y
            error_z = actual_z - planned_z
            error_mag = np.sqrt(error_x**2 + error_y**2 + error_z**2)
            
            self.position_error['x'].append(error_x)
            self.position_error['y'].append(error_y)
            self.position_error['z'].append(error_z)
            self.position_error['magnitude'].append(error_mag)
            
            # 输出误差信息
            if error_mag > 0.01:  # 误差大于10mm时输出警告
                self.get_logger().warn(f'位置误差较大: {error_mag*1000:.1f}mm')
    
    def tf_monitor_loop(self):
        """TF监控循环"""
        while rclpy.ok():
            try:
                # 获取末端执行器位置
                transform = self.tf_buffer.lookup_transform(
                    'base_link', 'link6', 
                    rclpy.time.Time(), 
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                
                current_time = time.time()
                
                with self.data_lock:
                    self.time_stamps.append(current_time)
                    
                    # 存储实际位置
                    self.actual_pos['x'].append(transform.transform.translation.x)
                    self.actual_pos['y'].append(transform.transform.translation.y)
                    self.actual_pos['z'].append(transform.transform.translation.z)
                    
            except Exception as e:
                pass  # 忽略TF查找失败
                
            time.sleep(0.1)  # 10Hz
    
    def get_plot_data(self):
        """获取绘图数据"""
        with self.data_lock:
            if len(self.time_stamps) == 0:
                return None
                
            # 转换时间戳为相对时间
            base_time = self.time_stamps[0] if self.time_stamps else 0
            times = [t - base_time for t in self.time_stamps]
            
            data = {
                'times': times,
                'actual_pos': {k: list(v) for k, v in self.actual_pos.items()},
                'planned_pos': {k: list(v) for k, v in self.planned_pos.items()},
                'position_error': {k: list(v) for k, v in self.position_error.items()}
            }
            
            return data

class PositionPlotter:
    def __init__(self, monitor):
        self.monitor = monitor
        
        # 创建图形窗口
        self.fig, self.axes = plt.subplots(2, 2, figsize=(14, 10))
        self.fig.suptitle('实时位置监控 - 实际位置 vs 规划位置', fontsize=16)
        
        # 设置子图标题
        titles = [
            'X轴位置对比 (m)', 'Y轴位置对比 (m)',
            'Z轴位置对比 (m)', '位置误差 (mm)'
        ]
        
        for i, ax in enumerate(self.axes.flat):
            ax.set_title(titles[i])
            ax.grid(True)
            ax.set_xlabel('时间 (s)')
        
        # 初始化线条
        self.lines = {}
        colors = ['blue', 'red', 'green']
        axes_names = ['x', 'y', 'z']
        
        # X, Y, Z轴位置对比
        for i, axis in enumerate(axes_names):
            ax = self.axes[i//2, i%2]
            
            # 实际位置（实线）
            line, = ax.plot([], [], color=colors[i], label=f'实际位置', linewidth=2)
            self.lines[f'actual_{axis}'] = line
            
            # 规划位置（虚线）
            line, = ax.plot([], [], color=colors[i], linestyle='--', label=f'规划位置', linewidth=2)
            self.lines[f'planned_{axis}'] = line
            
            ax.legend()
        
        # 位置误差
        ax = self.axes[1, 1]
        for i, axis in enumerate(axes_names):
            line, = ax.plot([], [], color=colors[i], label=f'{axis}轴误差', linewidth=1.5)
            self.lines[f'error_{axis}'] = line
        
        # 误差幅值
        line, = ax.plot([], [], color='black', label='误差幅值', linewidth=2)
        self.lines['error_magnitude'] = line
        ax.legend()
        
        plt.tight_layout()
        
    def update_plots(self, frame):
        """更新图形"""
        data = self.monitor.get_plot_data()
        if data is None or len(data['times']) == 0:
            return list(self.lines.values())
        
        times = data['times']
        
        # 更新位置对比
        for axis in ['x', 'y', 'z']:
            # 实际位置
            if axis in data['actual_pos'] and len(data['actual_pos'][axis]) > 0:
                actual_data = data['actual_pos'][axis]
                plot_times = times[-len(actual_data):]
                self.lines[f'actual_{axis}'].set_data(plot_times, actual_data)
            
            # 规划位置
            if axis in data['planned_pos'] and len(data['planned_pos'][axis]) > 0:
                planned_data = data['planned_pos'][axis]
                plot_times = times[-len(planned_data):]
                self.lines[f'planned_{axis}'].set_data(plot_times, planned_data)
        
        # 更新位置误差（转换为mm）
        for axis in ['x', 'y', 'z']:
            if axis in data['position_error'] and len(data['position_error'][axis]) > 0:
                error_data = [e * 1000 for e in data['position_error'][axis]]  # 转换为mm
                plot_times = times[-len(error_data):]
                self.lines[f'error_{axis}'].set_data(plot_times, error_data)
        
        # 更新误差幅值
        if 'magnitude' in data['position_error'] and len(data['position_error']['magnitude']) > 0:
            error_mag_data = [e * 1000 for e in data['position_error']['magnitude']]  # 转换为mm
            plot_times = times[-len(error_mag_data):]
            self.lines['error_magnitude'].set_data(plot_times, error_mag_data)
        
        # 自动调整坐标轴范围
        for ax in self.axes.flat:
            try:
                ax.relim()
                ax.autoscale_view()
            except:
                pass
        
        return list(self.lines.values())

def main():
    rclpy.init()
    
    # 创建监控器
    monitor = PositionMonitor()
    
    # 在单独线程中运行ROS
    ros_thread = threading.Thread(target=lambda: rclpy.spin(monitor), daemon=True)
    ros_thread.start()
    
    # 等待数据
    print("启动位置监控器，等待数据...")
    time.sleep(2)
    
    # 创建绘图器
    plotter = PositionPlotter(monitor)
    
    # 开始实时绘图
    print("开始实时绘图...")
    ani = FuncAnimation(plotter.fig, plotter.update_plots, interval=100, blit=False)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        print("用户中断程序")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 