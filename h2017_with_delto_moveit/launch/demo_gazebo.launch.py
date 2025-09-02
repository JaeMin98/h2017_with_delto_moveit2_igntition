import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Package directories
    pkg_share = FindPackageShare('h2017_with_delto_moveit').find('h2017_with_delto_moveit')
    gazebo_ros_pkg_share = FindPackageShare('gazebo_ros').find('gazebo_ros')

    # Paths to files
    urdf_xacro_path = PathJoinSubstitution([pkg_share, 'config', 'h2017_with_delto_gripper.urdf.xacro'])
    srdf_path = PathJoinSubstitution([pkg_share, 'config', 'h2017_with_delto_gripper.srdf'])
    moveit_controllers_path = PathJoinSubstitution([pkg_share, 'config', 'moveit_controllers.yaml'])
    ros2_controllers_path = PathJoinSubstitution([pkg_share, 'config', 'ros2_controllers.yaml'])
    joint_limits_path = PathJoinSubstitution([pkg_share, 'config', 'joint_limits.yaml'])
    kinematics_path = PathJoinSubstitution([pkg_share, 'config', 'kinematics.yaml'])
    pilz_cartesian_limits_path = PathJoinSubstitution([pkg_share, 'config', 'pilz_cartesian_limits.yaml'])
    rviz_config_path = PathJoinSubstitution([pkg_share, 'config', 'moveit.rviz'])

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_ros_pkg_share, '/launch/gazebo.launch.py']),
        launch_arguments={'verbose': 'false'}.items(),
    )

    robot_description_content = ParameterValue(Command(['xacro ', urdf_xacro_path]), value_type=str)
    robot_description_semantic_content = ParameterValue(Command(['cat ', srdf_path]), value_type=str)

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'h2017_with_delto_gripper'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_content}],
    )

    # Load controllers
    load_controllers = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_trajectory_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_action_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        )
    ]

    # MoveIt MoveGroup Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'robot_description_semantic': robot_description_semantic_content},
            kinematics_path,
            {'trajectory_execution': {'moveit_manage_controllers': True}},
            joint_limits_path,
            moveit_controllers_path,
            pilz_cartesian_limits_path,
            {'use_sim_time': use_sim_time},
        ],
    )

    # RViz with MoveIt configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_path],
        parameters=[
            {'robot_description': robot_description_content},
            {'robot_description_semantic': robot_description_semantic_content},
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time if true'),

        gazebo_launch,
        robot_state_publisher,
        spawn_entity,
        *load_controllers,
        move_group_node,
        rviz_node,
    ])
