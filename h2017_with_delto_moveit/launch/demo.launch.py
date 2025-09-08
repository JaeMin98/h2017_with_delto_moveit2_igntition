from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("h2017_with_delto_gripper", package_name="h2017_with_delto_moveit").to_moveit_configs()
    
    # Gazebo 런치 포함
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('h2017_with_delto_moveit'), 'launch', 'gazebo.launch.py')
        ])
    )
    
    # 브리지 런치 포함 (아래 새 파일 생성 필요)
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('h2017_with_delto_moveit'), 'launch', 'bridge.launch.py')
        ])
    )
    
    # 기존 MoveIt 데모 런치
    demo_ld = generate_demo_launch(moveit_config)
    
    # 모든 것을 합침 (use_sim_time=true로 설정)
    demo_ld.add_action(gazebo_launch)
    demo_ld.add_action(bridge_launch)
    
    # MoveItConfig에 sim_time 추가 (필요 시)
    # moveit_config.planning_pipelines['ompl']['planning_scene_monitor']['use_sim_time'] = True
    # moveit_config.planning_pipelines['ompl']['move_group']['use_sim_time'] = True
    
    return demo_ld