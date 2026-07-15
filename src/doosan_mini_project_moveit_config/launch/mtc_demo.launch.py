import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config_package_name = "doosan_mini_project_moveit_config"
    moveit_config_package_path = get_package_share_directory(
        moveit_config_package_name
    )

    # ==================================================================
    # Launch arguments
    # ==================================================================
    declared_arguments = [
        DeclareLaunchArgument(
            "gripper",
            default_value="none",
            description="Gripper type passed to e0509.urdf.xacro",
        ),
    ]

    gripper = LaunchConfiguration("gripper")

    # ==================================================================
    # MoveIt configuration
    # ==================================================================
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="e0509",
            package_name=moveit_config_package_name,
        )
        .robot_description(
            file_path="config/e0509.urdf.xacro",
            mappings={
                "gripper": gripper,
            },
        )
        .robot_description_semantic(
            file_path="config/e0509.srdf",
        )
        .robot_description_kinematics(
            file_path="config/kinematics.yaml",
        )
        .planning_pipelines(
            pipelines=["ompl"],
            default_planning_pipeline="ompl",
        )
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .to_moveit_configs()
    )

    # ==================================================================
    # Static TF
    #
    # URDF 안에 world -> base_link 연결이 이미 존재한다면 제거한다.
    # ==================================================================
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base_link_tf",
        output="log",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "world",
            "--child-frame-id", "base_link",
        ],
    )

    # ==================================================================
    # Robot State Publisher
    #
    # robot_description을 발행하고 /joint_states를 기반으로 TF를 생성한다.
    # ==================================================================
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
        ],
    )

    # ==================================================================
    # ros2_control
    # ==================================================================
    ros2_controllers_file = os.path.join(
        moveit_config_package_path,
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="both",
        parameters=[
            ros2_controllers_file,
        ],
        remappings=[
            (
                "/controller_manager/robot_description",
                "/robot_description",
            ),
        ],
    )

    # ==================================================================
    # Controller spawners
    # ==================================================================
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster_spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
            "--param-file",
            ros2_controllers_file,
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="arm_controller_spawner",
        output="screen",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
            "--param-file",
            ros2_controllers_file,
        ],
    )

    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="hand_controller_spawner",
        output="screen",
        arguments=[
            "hand_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
            "--param-file",
            ros2_controllers_file,
        ],
    )

    # ==================================================================
    # MoveGroup
    #
    # ExecuteTaskSolutionCapability:
    # MTC에서 생성한 전체 Task Solution을 실행하기 위한 capability다.
    #
    # 외부 MTC 노드에서 다음과 같이 실행할 때 사용된다.
    #
    # task.execute(*task.solutions().front());
    # ==================================================================
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "capabilities":
                    "move_group/ExecuteTaskSolutionCapability",
            },
        ],
    )

    # ==================================================================
    # RViz configuration
    #
    # config/mtc.rviz가 있으면 우선 사용한다.
    # 없으면 Setup Assistant가 생성한 config/moveit.rviz를 사용한다.
    # ==================================================================
    mtc_rviz_config_file = os.path.join(
        moveit_config_package_path,
        "config",
        "mtc.rviz",
    )

    default_rviz_config_file = os.path.join(
        moveit_config_package_path,
        "config",
        "moveit.rviz",
    )

    if os.path.exists(mtc_rviz_config_file):
        rviz_config_file = mtc_rviz_config_file
    else:
        rviz_config_file = default_rviz_config_file

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            rviz_config_file,
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )



    # ==================================================================
    # Launch description
    # ==================================================================
    return LaunchDescription(
        declared_arguments
        + [
            static_tf_node,
            robot_state_publisher_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            hand_controller_spawner,
            move_group_node,
            rviz_node,
        ]
    )