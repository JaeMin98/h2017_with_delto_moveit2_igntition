# 1. 필요한 라이브러리 임포트
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit # OnProcessStart 추가
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # =============================================================================
    # === 1. 설정 변수 및 Launch Argument 선언 ===
    # =============================================================================
    # URDF/Xacro 파일이 있는 패키지 이름
    pkg_name = 'h2017_with_delto_moveit'
    # 패키지 내의 URDF/Xacro 파일 상대 경로 (기존)
    file_subpath = 'config/h2017_with_delto_gripper.urdf'
    # Gazebo 내에서 사용할 로봇 이름
    robot_name_in_gazebo = 'h2017'
    # 시뮬레이션 시간 사용 여부
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # =============================================================================
    # === 2. Gazebo 리소스 경로 설정 ===
    # =============================================================================
    pkg_share_path = get_package_share_directory(pkg_name)
    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    new_gz_resource_path = f"{pkg_share_path}:{gz_resource_path}"
    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=new_gz_resource_path
    )
    
    # =============================================================================
    # === 3. URDF/Xacro 파일 처리 ===
    # =============================================================================
    # URDF/Xacro 파일의 절대 경로 생성 (pkg_share_path 기반)
    urdf_file_path = os.path.join(pkg_share_path, file_subpath)
    print(f"Loading robot description from: {urdf_file_path}")
    robot_description_raw = ""

    # 확장자가 .xacro이면 xacro로 처리, 그렇지 않으면 일반 파일로 읽음
    if urdf_file_path.endswith('.xacro'):
        robot_description_raw = xacro.process_file(urdf_file_path).toxml()
    else:
        with open(urdf_file_path, 'r') as f:
            robot_description_raw = f.read()


    # =============================================================================
    # === 4. Robot State Publisher 노드 설정 ===
    # =============================================================================
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': use_sim_time
        }]
    )

    # =============================================================================
    # === 5. Gazebo Ignition (Gazebo Sim) 실행 ===
    # =============================================================================
    gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # =============================================================================
    # === 6. Gazebo Ignition 스폰 노드 설정 ===
    # =============================================================================
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', 
            '-name', robot_name_in_gazebo,
            '-allow_renaming', 'true',
            '-x', '0.0', '-y', '0.0', '-z', '0.1'  # 초기 포즈 추가
        ],
        output='screen'
    )

    # =============================================================================
    # === 7. ros2_control 컨트롤러 매니저 ===
    # =============================================================================
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description_raw},
                    os.path.join(pkg_share_path, 'config', 'ros2_controllers.yaml')],
        output="screen",
        remappings=[('/controller_manager/robot_description', '/robot_description'),
                    ('/controller_manager/tf', '/tf')],
    )

    # 스포너 노드들
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_action_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # =============================================================================
    # === 8. LaunchDescription 구성 및 반환 (수정된 이벤트 핸들러 적용) ===
    # =============================================================================
    return LaunchDescription([
        set_gazebo_resource_path,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        gazebo,
        node_robot_state_publisher,
        controller_manager_node,
        
        # *** 수정된 부분 ***
        # controller_manager_node가 시작(OnProcessStart)되면 스포너들을 실행
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager_node,
                on_start=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager_node,
                on_start=[arm_controller_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager_node,
                on_start=[gripper_controller_spawner],
            )
        ),
        spawn_entity
    ])