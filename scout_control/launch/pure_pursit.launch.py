from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[{
                'controller_plugins': ['follow_path'],
                'follow_path.plugin': 'nav2_pure_pursuit_controller::PurePursuitController',
                'follow_path.desired_linear_vel': 0.5,
                'follow_path.lookahead_dist': 0.6,
                'use_sim_time': True,
            }],
            remappings=[
                ('/cmd_vel', '/diff_drive_base_controller/cmd_vel_unstamped'),
            ]
        ),
        Node(
            package='nav2_simple_commander',
            executable='nav2_simple_commander',
            name='path_publisher',
            output='screen',
            parameters=[{
                'path_file': '/home/orin/scout_path.csv',
                'use_sim_time': True,
            }]
        )
    ])
