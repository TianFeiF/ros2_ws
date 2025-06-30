#!/usr/bin/env python3
"""
简单的轨迹监听测试脚本
用于验证是否能正确接收MoveIt的规划轨迹数据
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory

class TrajectoryListener(Node):
    def __init__(self):
        super().__init__('trajectory_listener')
        
        # 订阅规划轨迹
        self.trajectory_subscription = self.create_subscription(
            DisplayTrajectory,
            '/display_planned_path',
            self.trajectory_callback,
            10
        )
        
        self.get_logger().info('轨迹监听器启动，等待规划轨迹数据...')
        self.get_logger().info('话题: /display_planned_path')
        
    def trajectory_callback(self, msg):
        """处理规划轨迹消息"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('收到规划轨迹消息！')
        
        if not msg.trajectory or len(msg.trajectory) == 0:
            self.get_logger().warn('轨迹消息为空')
            return
            
        # 获取机器人轨迹
        robot_trajectory = msg.trajectory[0]
        if not robot_trajectory.joint_trajectory.points:
            self.get_logger().warn('关节轨迹点为空')
            return
        
        self.get_logger().info(f'轨迹包含 {len(robot_trajectory.joint_trajectory.points)} 个点')
        self.get_logger().info(f'关节名称: {robot_trajectory.joint_trajectory.joint_names}')
        
        # 获取第一个点和最后一个点
        first_point = robot_trajectory.joint_trajectory.points[0]
        last_point = robot_trajectory.joint_trajectory.points[-1]
        
        self.get_logger().info('起始关节位置:')
        for i, pos in enumerate(first_point.positions):
            self.get_logger().info(f'  joint{i+1}: {pos:.3f} rad')
            
        self.get_logger().info('目标关节位置:')
        for i, pos in enumerate(last_point.positions):
            self.get_logger().info(f'  joint{i+1}: {pos:.3f} rad')
            
        self.get_logger().info('=' * 50)

def main():
    rclpy.init()
    
    # 创建监听器
    listener = TrajectoryListener()
    
    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        print("用户中断程序")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 