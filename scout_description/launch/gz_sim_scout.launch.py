from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction


def generate_launch_description():
    pkg_scout = get_package_share_directory('scout_description')
    pkg_world = get_package_share_directory('lidar_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_controllers = get_package_share_directory('scout_control')

    default_model = PathJoinSubstitution([pkg_scout, 'urdf', 'scout_mini.urdf.xacro'])
    default_world = PathJoinSubstitution([pkg_world, 'worlds', 'lidar_world.sdf'])

    robot_controllers = PathJoinSubstitution([pkg_controllers, 'config', 'scout_mini_controller.yaml'])

    sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='If true, use simulated clock'
    )

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
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_ros_gz_sim, '/launch/gz_sim.launch.py']),
        launch_arguments=[('gz_args', ['-r -v 4 ', default_world])]
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description', '-name', 'scout_mini', '-allow_renaming', 'true'],
        output='screen'
    )

    # --------------- 控制器 spawner ---------------
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60'],
        parameters=[{'use_sim_time': True}]
    )

    diff_drive = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_base_controller',
                   '--controller-manager', '/controller_manager',
                   '--param-file', robot_controllers],
        parameters=[{'use_sim_time': True}]
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # LiDAR
            '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            # RGBD Camera
            '/rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/rgbd_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            # IMU
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # static TF
    tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["0", "0", "0", "0", "0", "0", "lidar_link", "scout_mini/base_link/gpu_lidar"],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    tf_rgbd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "rgbd_camera_link", "scout_mini/base_link/rgbd_camera"],
        parameters=[{'use_sim_time': True}],
        output="screen"
    )

    tf_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0.2", "0", "0", "0", "imu_link", "scout_mini/base_link/imu_sensor"],
        parameters=[{'use_sim_time': True}],
        output="screen"
    )

    tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # LiDAR 重映射节点
    lidar_remap_node = Node(
        package='topic_tools',
        executable='relay',
        name='lidar_relay',
        arguments=['/lidar', '/scan'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    points_remap_node = Node(
        package='topic_tools',
        executable='relay',
        name='points_relay',
        arguments=['/lidar/points', '/scan/points'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    slam_toolbox_node = TimerAction(
        period=15.0,  # 延迟 10 秒启动，确保 TF 和 /scan 已经稳定
        actions=[Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                {
                    'use_sim_time': True,
                    'scan_queue_size': 50,
                    'scan_topic': '/scan',
                    'transform_timeout': 0.5,  # 增大缓存容忍时间，防止 extrapolation 报错
                    'odom_frame': 'odom',      # 确保使用正确的 odom frame
                    'base_frame': 'base_link',
                    'map_frame': 'map'
                }
            ],
            remappings=[('/odom', '/diff_drive_base_controller/odom')],
            output='screen',
            arguments=['--ros-args', '--log-level', 'slam_toolbox:=debug']  # 启用调试日志
        )]
    )

    return LaunchDescription([
        declare_model,
        bridge,
        tf_lidar,
        tf_rgbd,
        tf_imu,
        # tf_odom,
        robot_state_pub,
        gz_sim,
        spawn,
        RegisterEventHandler(
            OnProcessExit(target_action=spawn, on_exit=[joint_state_broadcaster])
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=joint_state_broadcaster, on_exit=[diff_drive])
        ),
        points_remap_node,
        lidar_remap_node,
        slam_toolbox_node,
        sim_time,
    ])

