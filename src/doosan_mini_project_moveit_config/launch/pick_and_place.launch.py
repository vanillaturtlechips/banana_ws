import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = "doosan_mini_project_moveit_config"

    package_share_directory = get_package_share_directory(package_name)
    launch_directory = os.path.join(package_share_directory, "launch")

    # 1. MoveIt2 + MTC + RViz 환경 실행
    mtc_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                launch_directory,
                "mtc_demo.launch.py",
            )
        )
    )

    # 2. 비전 좌표 수신 및 MTC Task 생성 노드 실행
    pick_and_place_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                launch_directory,
                "pick_and_place_system.launch.py",
            )
        )
    )

    # 3. /solution 토픽을 JSON 파일로 저장하는 노드 실행
    mtc_solution_json_saver_node = Node(
        package=package_name,
        executable="mtc_solution_json_saver",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
            }
        ],
    )

    return LaunchDescription([
        mtc_demo_launch,
        pick_and_place_system_launch,
        mtc_solution_json_saver_node,
    ])