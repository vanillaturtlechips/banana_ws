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
                # ⚠️ 2π면 grasp 방향을 1개만 샘플 → 그 하나가 IK 실패하면 solutions=0.
                #    작게(0.2rad≈11.5°) 주면 접근축 둘레로 ~31개 방향을 뿌려 IK 성공률↑
                # ⚡ 속도 튜닝: 계획이 300초씩 걸려 사실상 펜딩. 탐색을 줄여 ~30초 내로.
                #   grasp 후보 0.4rad(~16개, 절반) / IK 해 줄임 / connect 1.5s / 해 1개면 종료.
                "grasp_angle_delta": 0.4,
                "pick_max_ik_solutions": 2,
                "place_max_ik_solutions": 1,
                "ik_min_solution_distance": 0.1,
                "connect_timeout_sec": 1.5,
                # 해 1개만 찾으면 즉시 종료(여러 개 탐색이 300초 주범)
                "max_task_solutions": 1,
                # grasp 접근 기울기(도): 0=수직(top-down). 실물에서 수직 잡기 확인됨.
                "grasp_pitch_deg": 0.0,
                # 🗑️ 보관함 좌표 (world 기준). ⚠️ 실물 보관함 위치 재서 교체.
                #   정책: ripe/overripe→bin1, rotten→bin2, unripe는 안 집음
                "bin1_x": 0.335, "bin1_y": -0.156,  # ripe/overripe 바구니 실측
                "bin2_x": 0.351, "bin2_y": 0.117,   # rotten 바구니 실측
                # 🧱 안전 충돌체(테이블/보관함). 실물 위치 확정 후 true + 아래 실측값으로.
                #    지금은 placeholder라 OFF(켜면 잘못된 위치가 계획을 막을 수 있음).
                "add_collision_scene": False,
                "table_z": 0.0,          # 테이블 윗면 world z (실측)
                "bin_size": 0.15,        # 보관함 한 변(m)
                "bin_wall_height": 0.10,
            },
        ],
        # 🔍 진단용 로그 레벨: FCL 충돌 검사 상세(어느 링크쌍을 검사/충돌하는지)
        arguments=[
            "--ros-args",
            "--log-level", "moveit_collision_detection_fcl.collision_common:=DEBUG",
        ],
    )

    return LaunchDescription([
        pick_place_system,
    ])