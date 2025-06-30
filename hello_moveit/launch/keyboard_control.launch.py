from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动键盘输入发布节点
        Node(
            package='hello_moveit',
            executable='keyboard_publisher',
            name='keyboard_publisher',
            output='screen',
            prefix='gnome-terminal --'  # 在新终端中启动键盘节点
        ),
        
        # 启动MoveIt运动规划订阅节点
        Node(
            package='hello_moveit',
            executable='moveit_subscriber',
            name='moveit_subscriber',
            output='screen'
        )
    ]) 