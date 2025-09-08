from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction, SetEnvironmentVariable
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 패키지 경로
    pkg = FindPackageShare('h2017_with_delto_moveit')
    ign = FindPackageShare('ros_ign_gazebo')

    # GAZEBO_MODEL_PATH 설정
    set_gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        [EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''), ':', pkg]
    )

    # 주요 파일 경로
    urdf = PathJoinSubstitution([pkg, 'config', 'h2017_with_delto_gripper.urdf'])
    srdf = PathJoinSubstitution([pkg, 'config', 'h2017_with_delto_gripper.srdf'])
    moveit_ctrls = PathJoinSubstitution([pkg, 'config', 'moveit_controllers.yaml'])
    joint_limits = PathJoinSubstitution([pkg, 'config', 'joint_limits.yaml'])
    kinematics = PathJoinSubstitution([pkg, 'config', 'kinematics.yaml'])
    pilz_limits = PathJoinSubstitution([pkg, 'config', 'pilz_cartesian_limits.yaml'])
    rviz_cfg = PathJoinSubstitution([pkg, 'config', 'moveit.rviz'])
    controller_config = PathJoinSubstitution([pkg, 'config', 'ros2_controllers.yaml'])

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 1) Ignition Gazebo 실행 (빈 월드)
    ign_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign, '/launch/ign_gazebo.launch.py']),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # 2) 로봇 description (URDF → /robot_description)
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf]),
        value_type=str
    )
    robot_semantic = ParameterValue(Command(['cat', ' ', srdf]), value_type=str)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}],
        output='screen'
    )

    # 3) ros2_control_node 실행
    ros2_control = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'use_sim_time': use_sim_time},
                    {'robot_description': robot_description},
                    controller_config],
        output='screen'
    )

    # 4) Ignition에 로봇 스폰
    spawn = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'h2017_with_delto_gripper',
                   '-allow_renaming', 'true'],
        output='screen'
    )

    # 5) 컨트롤러 순차 스폰 (JSB → Arm → Gripper)
    sp_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '120'],
        output='screen'
    )
    sp_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_trajectory_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '120'],
        output='screen'
    )
    sp_grp = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_action_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '120'],
        output='screen'
    )

    order_jsb = RegisterEventHandler(
        OnProcessStart(target_action=spawn,
                       on_start=[TimerAction(period=2.0, actions=[sp_jsb])])
    )
    order_arm = RegisterEventHandler(
        OnProcessStart(target_action=sp_jsb,
                       on_start=[sp_arm])
    )
    order_grp = RegisterEventHandler(
        OnProcessStart(target_action=sp_arm,
                       on_start=[sp_grp])
    )

    # 6) MoveIt move_group
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description},
            {'robot_description_semantic': robot_semantic},
            kinematics, joint_limits, pilz_limits,
            {'trajectory_execution': {'moveit_manage_controllers': True}},
            moveit_ctrls
        ],
    )

    # 7) RViz2 실행
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_cfg],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description},
            {'robot_description_semantic': robot_semantic}
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        set_gazebo_model_path,
        ign_world,
        rsp,
        ros2_control,
        spawn,
        order_jsb, order_arm, order_grp,
        move_group,
        rviz
    ])
