import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import xacro

def generate_launch_description():
    # 시뮬레이션 시간 사용 여부 설정 (Gazebo와 연동 시 필수)
    use_sim_time = {'use_sim_time': True}

    # 1. 로봇 URDF 로드
    robot_description_config = xacro.process_file(
        os.path.join(
            FindPackageShare('h2017_with_delto_moveit').find('h2017_with_delto_moveit'),
            'config',
            'h2017_with_delto_gripper.urdf.xacro'
        )
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    # 2. robot_state_publisher 노드 실행
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, use_sim_time]
    )

    # 3. Gazebo 실행
    # gz_server = ExecuteProcess(
    #     cmd=['ign', 'gazebo', '-r', 'empty.sdf'],
    #     output='screen'

    # gz_client = ExecuteProcess(
    #     cmd=['ign', 'gazebo', '-g'],
    #     output='screen'
    # )

    # 4. Gazebo에 로봇 스폰
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen'
    )

    # 5. 컨트롤러 스포너(spawner) 실행
    # joint_state_broadcaster 로드
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    # arm_trajectory_controller 로드
    spawn_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    # gripper_action_controller 로드
    spawn_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_action_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # 6. MoveIt2 실행
    moveit_config = (
        MoveItConfigsBuilder("h2017", package_name="h2017_with_delto_moveit")
       .robot_description(file_path="/home/smartcps/ros2_ws/src/h2017_with_delto_moveit/config/h2017_with_delto_gripper.urdf.xacro")
       .trajectory_execution(file_path="/home/smartcps/ros2_ws/src/h2017_with_delto_moveit/config/moveit_controllers.yaml")
        #... 기타 MoveIt 설정...
       .to_moveit_configs()
    )
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), use_sim_time],
    )

    # 7. RViz 실행
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", "/home/smartcps/ros2_ws/src/h2017_with_delto_moveit/config/moveit.rviz"],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            use_sim_time
        ],
    )

    return LaunchDescription([
        # gz_server,
        # gz_client,
        robot_state_publisher_node,
        spawn_entity,
        spawn_jsb,
        spawn_arm_controller,
        spawn_gripper_controller,
        move_group_node,
        rviz_node
    ])
