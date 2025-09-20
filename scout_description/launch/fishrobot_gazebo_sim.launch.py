from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_scout = get_package_share_directory('scout_description')
    pkg_world = get_package_share_directory('sim_world')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    default_model = PathJoinSubstitution([pkg_scout, 'urdf', 'fishros_robot.urdf.xacro'])
    default_world = PathJoinSubstitution([pkg_world, 'worlds', 'custom_room.world'])

    declare_model = DeclareLaunchArgument(
        'model', default_value=default_model, description='URDF path'
    )

    robot_desc = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), value_type=str
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='',
        parameters=[{'robot_description': robot_desc}]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_ros_gz_sim, '/launch/gz_sim.launch.py']),
        launch_arguments=[('gz_args', ['-r ', default_world])]
        # launch_arguments={'gz_args': ['-r', default_world]}.items()
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description', '-name', 'fishbot', '-allow_renaming', 'true'],
        output='screen'
    )


    # --------------- 控制器 spawner（Humble 推荐写法） ---------------
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fishbot_joint_state_broadcaster', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60']
    )

    diff_drive = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fishbot_diff_drive_controller', '--controller-manager', '/controller_manager']
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # '/front_lidar/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/front_lidar/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/base_imu/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'],
        output='screen'
    )

    return LaunchDescription([
        declare_model,
        bridge,
        robot_state_pub,
        gz_sim,
        spawn,
        # 等机器人 spawn 完再加载控制器
        RegisterEventHandler(
            OnProcessExit(target_action=spawn, on_exit=[joint_state_broadcaster])
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=joint_state_broadcaster, on_exit=[diff_drive])
        ),
    ])
