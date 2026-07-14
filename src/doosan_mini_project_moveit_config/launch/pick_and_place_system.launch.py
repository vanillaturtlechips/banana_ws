from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "e0509",
            package_name="doosan_mini_project_moveit_config",
        )
        .robot_description()
        .robot_description_semantic()
        .robot_description_kinematics()
        .joint_limits()
        .trajectory_execution()
        .planning_pipelines(
            pipelines=["ompl"],
            default_planning_pipeline="ompl",
        )
        .to_moveit_configs()
    )

    pick_place_system = Node(
        package="doosan_mini_project_moveit_config",
        executable="pick_place_system",
        # 하나의 executable 안에서 mtc_task_node와 pick_place_command_node를
        # 생성하므로 여기서는 name을 강제로 지정하지 않는다.
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": False,

                # MTC 탐색량 제한 기본값
                "grasp_angle_delta": 6.283185307179586,
                # 정확도가 떨어지는 경우 수정 1
                "pick_max_ik_solutions": 2,
                #
                "place_max_ik_solutions": 1,
                "ik_min_solution_distance": 0.1,
                # 정확도가 떨어지는 경우 수정 2
                "connect_timeout_sec": 3.0,
                #
                "max_task_solutions": 1,
            },
        ],
    )

    return LaunchDescription([
        pick_place_system,
    ])