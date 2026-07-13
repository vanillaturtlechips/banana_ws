"""perception 노드 실행. use_fake_camera:=true 면 가짜 카메라도 같이 띄운다."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    cfg = os.path.join(
        get_package_share_directory("banana_perception"), "config", "perception.yaml")
    use_fake = LaunchConfiguration("use_fake_camera")

    return LaunchDescription([
        DeclareLaunchArgument("use_fake_camera", default_value="true",
                              description="합성 카메라 노드도 실행"),
        Node(package="banana_perception", executable="perception",
             parameters=[cfg], output="screen"),
        Node(package="banana_perception", executable="fake_camera",
             condition=IfCondition(use_fake), output="screen"),
    ])
