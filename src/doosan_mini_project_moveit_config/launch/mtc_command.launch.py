from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "e0509",
            package_name="doosan_mini_project_moveit_config",
        )
        .to_moveit_configs()
    )

    mtc_node = Node(
        package="doosan_mini_project_moveit_config",
        executable="mtc_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    return LaunchDescription([
        mtc_node,
    ])