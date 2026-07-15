import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ####################################################################
    # Doosan Bringup Launch
    ####################################################################
    dsr_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("dsr_bringup2"),
                "launch",
                "dsr_bringup2_rviz.launch.py",
            )
        ),
        launch_arguments={
            "mode": "real",
            "host": "110.120.1.13",
            "model": "e0509",
        }.items(),
    )

    ####################################################################
    # Gripper Service
    ####################################################################
    gripper_service_node = Node(
        package="dsr_gripper",
        executable="gripper_service",
        name="gripper_service",
        output="screen",
    )

    ####################################################################
    # Launch
    ####################################################################
    return LaunchDescription([
        dsr_bringup_launch,
        gripper_service_node,
    ])