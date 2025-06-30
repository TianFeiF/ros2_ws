from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 键盘输入节点 - 多按键检测模式
        Node(
            package='hello_moveit',
            executable='keyboard_publisher',
            name='keyboard_publisher',
            output='screen',
            parameters=[],
        ),
        
        # MoveIt运动规划节点 - 支持多按键同时处理
        Node(
            package='hello_moveit',
            executable='moveit_subscriber',
            name='moveit_subscriber',
            output='screen',
            parameters=[],
        ),
    ]) 