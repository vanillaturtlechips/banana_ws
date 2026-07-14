import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config_package_name = "doosan_mini_project_moveit_config"

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    declared_arguments = [
        DeclareLaunchArgument(
            "gripper",
            default_value="none",
            description="Gripper type",
        ),
    ]

    gripper = LaunchConfiguration("gripper")

    # ------------------------------------------------------------------
    # MoveIt configuration
    #
    # config/e0509.urdf.xacro
    # config/e0509.srdf.xacro
    # config/moveit_controllers.yaml
    # config/kinematics.yaml
    # config/ompl_planning.yaml
    # 등을 읽는다.
    # ------------------------------------------------------------------
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
            mappings={
                "gripper": gripper,
            },
        )
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .to_moveit_configs()
    )

    # ------------------------------------------------------------------
    # MoveGroup
    # ------------------------------------------------------------------
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    # ------------------------------------------------------------------
    # RViz
    # Setup Assistant에서 생성된 moveit.rviz는 일반적으로 config 폴더에 있다.
    # ------------------------------------------------------------------
    rviz_config_file = os.path.join(
        get_package_share_directory(moveit_config_package_name),
        "config",
        "moveit.rviz",
    )

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

    # ------------------------------------------------------------------
    # world -> base_link 고정 TF
    #
    # URDF에 이미 world 링크 또는 world -> base_link 조인트가 있다면
    # 이 노드는 제거해야 한다.
    # ------------------------------------------------------------------
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
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

    # ------------------------------------------------------------------
    # Robot State Publisher
    #
    # /joint_states와 robot_description을 이용해 TF를 발행한다.
    # Jazzy의 controller_manager도 /robot_description을 구독한다.
    # ------------------------------------------------------------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
        ],
    )

    # ------------------------------------------------------------------
    # ros2_control configuration
    # ------------------------------------------------------------------
    ros2_controllers_file = os.path.join(
        get_package_share_directory(moveit_config_package_name),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            ros2_controllers_file,
        ],
        remappings=[
            # Jazzy controller_manager가 구독하는 robot_description
            ("/robot_description", "/robot_description"),
        ],
    )

    # ------------------------------------------------------------------
    # ros2_control controller spawners
    #
    # ros2_controllers.yaml에 정의된 실제 이름을 사용해야 한다.
    # ------------------------------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
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