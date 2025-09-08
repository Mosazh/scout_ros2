# scout_teleop.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 键盘控制节点：必须用独立终端运行
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            remappings=[
                ('cmd_vel', '/cmd_vel_keyboard')
            ],
            output='screen',
            prefix='xterm -e',  # 关键！为节点开一个新终端
        ),

        # 话题中继
        Node(
            package='topic_tools',
            executable='relay',
            name='cmd_vel_relay',
            arguments=[
                '/cmd_vel_keyboard',
                '/diff_drive_base_controller/cmd_vel_unstamped'
            ],
            output='screen'
        )
    ])
