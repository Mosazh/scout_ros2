# 导入launch相关模块
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

# 导入ROS2 launch相关模块
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # 通过xacro工具获取URDF机器人描述文件
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

    # 将机器人描述内容封装为参数字典
    robot_description = {'robot_description': robot_description_content}

    # 获取差速驱动控制器配置文件路径
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('scout_control'),
            'config',
            'scout_mini_controller.yaml',
        ]
    )

    # 创建机器人状态发布节点
    # 该节点负责发布机器人各关节的状态信息
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Gazebo实体生成节点
    # 将机器人模型加载到Gazebo仿真环境中，命名为diff_drive
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'diff_drive', '-allow_renaming', 'true'],
    )

    # 关节状态广播器加载节点
    # 负责加载并启动关节状态广播控制器
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # 差速驱动基础控制器加载节点
    # 负责加载并启动差速驱动基础控制器，使用指定的控制器参数文件
    diff_drive_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_base_controller',
            '--param-file',
            robot_controllers,
            ],
    )

    # ROS-Gazebo桥接节点
    # 建立ROS和Gazebo之间的消息桥接，此处用于同步时钟消息
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # 返回Launch描述列表，定义了launch文件执行的所有操作
    return LaunchDescription([
        # 包含Gazebo仿真环境的launch文件
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 1 empty.sdf'])]),

        # 注册事件处理器：当gz_spawn_entity节点执行完毕后，
        # 启动joint_state_broadcaster_spawner节点
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),

        # 注册事件处理器：当joint_state_broadcaster_spawner节点执行完毕后，
        # 启动diff_drive_base_controller_spawner节点
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_base_controller_spawner],
            )
        ),

        # 添加桥接节点
        bridge,

        # 添加机器人状态发布节点
        node_robot_state_publisher,

        # 添加Gazebo实体生成节点
        gz_spawn_entity,

        # 声明launch参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])
