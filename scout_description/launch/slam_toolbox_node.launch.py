from launch.actions import TimerAction
from launch_ros.actions import Node
from launch import LaunchDescription
def generate_launch_description():
    slam_toolbox_node = TimerAction(
        period=1.0,
        actions=[Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[{'use_sim_time': True, 'scan_queue_size': 50, 'scan_topic': '/scan'}],
            output='screen',
            arguments=['--ros-args', '--log-level', 'slam_toolbox:=debug']  # ← 启用调试
        )]
    )

    return LaunchDescription([slam_toolbox_node])
