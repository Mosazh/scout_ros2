from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    #
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('scout_description'),
                 'urdf', 'scout_mini.urdf']
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    # Controller config file
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('scout_control'),
            'config',
            'scout_mini_controller.yaml',
        ]
    )

    # Robot state publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='scout_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Spawn robot in Gazebo
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'scout_mini',
            '-allow_renaming', 'true'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Bridge: Clock
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Spawner: joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', 'controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Spawner: diff_drive_controller (NOT joint_trajectory_controller!)
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        # arguments=['scout_mini_velocity_controller', '--param-file', robot_controllers],
        arguments=['scout_mini_velocity_controller', '--param-file', robot_controllers, '--controller-manager', 'controller_manager'],

        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        # Declare launch arg
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use Gazebo clock'),

        # Start Gazebo Sim with empty world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments={'gz_args': '-r -v 1 empty.sdf'}.items()
        ),

        # Robot state publisher
        node_robot_state_publisher,

        # Bridge clock
        bridge,

        # Spawn entity
        gz_spawn_entity,

        # On spawn exit → start joint_state_broadcaster
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),

        # On joint_state_broadcaster exit → start diff_drive_controller
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_controller_spawner],
            )
        ),
    ])
