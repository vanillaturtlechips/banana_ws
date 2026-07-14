import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
# import xacro

def generate_launch_description():

    package_name = "doosan_mini_project_cpp"
    urdf_file_name = "e0509_with_gripper.urdf"



    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg_path = os.path.join(get_package_share_directory(package_name))
    urdf_file = os.path.join(
        pkg_path,
        "models",
        "robot_arm",
        "urdf",
        urdf_file_name,
    )
    
    with open(urdf_file, "r") as f:
        robot_description = f.read()

    params = {
        "robot_description": robot_description,
        "use_sim_time": use_sim_time,
        }

    rviz_config_file = get_package_share_directory(package_name) + "/config/simple_load.rviz"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="use sim time"
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[params],
            ),
            Node(package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='log',
                arguments=['-d', rviz_config_file],
            ),
            Node(package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='rviz2',
                output="screen",
            ),
        ]
    )