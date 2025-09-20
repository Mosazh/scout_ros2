from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    pkg_scout = get_package_share_directory('scout_description')
    pkg_world = get_package_share_directory('sim_world')

    # URDF 路径（xacro 文件）
    xacro_file = os.path.join(pkg_scout, 'urdf', 'lidar_robot.urdf.xacro')
    # 转换为 urdf
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    # World 路径
    world_path = os.path.join(pkg_world, 'worlds', 'lidar_world.sdf')

    return LaunchDescription([

        # 启动 Ignition Gazebo
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', '-v', world_path],
            output='screen'
        ),

        # 发布机器人描述
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # 桥接 Lidar
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/front_lidar/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'],
            output='screen'
        ),
    ])
