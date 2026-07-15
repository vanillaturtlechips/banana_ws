#!/usr/bin/env python3

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Callable

import rclpy
import DR_init

from ament_index_python.packages import get_package_share_directory


# ============================================================
# Doosan 로봇 설정
# ============================================================

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"

PACKAGE_NAME = "doosan_mini_project_moveit_config"


# ============================================================
# 관절 이름 설정
# ============================================================

ARM_JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
]

GRIPPER_JOINT_NAMES = [
    "rh_l1",
    "rh_l2",
    "rh_p12_rn",
    "rh_r2",
]


# ============================================================
# 이동 속도 / 가속도 설정
# ============================================================

MOVEJ_VELOCITY = 30.0
MOVEJ_ACCELERATION = 60.0

MOVEL_VELOCITY = 100.0
MOVEL_ACCELERATION = 200.0


# ============================================================
# 그리퍼 설정
# ============================================================

GRIPPER_OPEN_POSITION = 750
GRIPPER_CLOSE_POSITION = 0

GRIPPER_OPEN_CURRENT = 200
GRIPPER_CLOSE_CURRENT = 300

GRIPPER_WAIT_TIME = 5.0

# JSON의 그리퍼 값이 모두 이 값 이하이면 열린 상태로 판단
GRIPPER_OPEN_THRESHOLD = 0.05


# ============================================================
# DSR API 지연 바인딩용 전역 변수
#
# DSR_ROBOT2는 import 순간 DR_init.__dsr__node를 참조하므로
# 파일 최상단에서 import하면 안 된다.
# ============================================================

movej: Callable[..., int] | None = None
movel: Callable[..., int] | None = None
fkin: Callable[..., Any] | None = None
posj: Callable[..., Any] | None = None
gripper_cmd: Callable[..., int] | None = None
DR_BASE: Any = None


def get_fixed_json_path() -> Path:
    """
    doosan_mini_project_moveit_config 패키지의 share 경로를 기준으로
    고정 JSON 파일 경로를 반환한다.

    <package_share>/
      trajectories/
        test/
          solution_trajectories.json
    """

    package_share_directory = Path(
        get_package_share_directory(PACKAGE_NAME)
    )

    return (
        package_share_directory
        / "trajectories"
        / "test"
        / "solution_trajectories.json"
    )


def load_trajectory_json(
    json_path: Path,
) -> dict[str, Any]:
    """JSON 파일을 읽고 기본 구조를 검증한다."""

    if not json_path.exists():
        raise FileNotFoundError(
            f"JSON 파일을 찾을 수 없습니다: {json_path}"
        )

    if not json_path.is_file():
        raise ValueError(
            f"지정된 경로가 파일이 아닙니다: {json_path}"
        )

    with json_path.open(
        "r",
        encoding="utf-8",
    ) as json_file:
        data = json.load(json_file)

    if not isinstance(data, dict):
        raise ValueError(
            "JSON 최상위 데이터는 객체여야 합니다."
        )

    trajectories = data.get("trajectories")

    if not isinstance(trajectories, list):
        raise ValueError(
            "JSON에 'trajectories' 배열이 없습니다."
        )

    return data


def parse_joint_data(
    trajectory_entry: dict[str, Any],
) -> dict[str, float]:
    """
    joint_names와 trajectory를 결합해
    {joint_name: position} 형태로 변환한다.
    """

    joint_names = trajectory_entry.get(
        "joint_names"
    )

    positions = trajectory_entry.get(
        "trajectory"
    )

    if not isinstance(joint_names, list):
        raise ValueError(
            "'joint_names'가 배열이 아닙니다."
        )

    if not isinstance(positions, list):
        raise ValueError(
            "'trajectory'가 배열이 아닙니다."
        )

    if len(joint_names) != len(positions):
        raise ValueError(
            "joint_names와 trajectory의 길이가 다릅니다. "
            f"joint_names={len(joint_names)}, "
            f"trajectory={len(positions)}"
        )

    joint_data: dict[str, float] = {}

    for joint_name, position in zip(
        joint_names,
        positions,
        strict=True,
    ):
        if not isinstance(joint_name, str):
            raise ValueError(
                f"관절 이름이 문자열이 아닙니다: "
                f"{joint_name}"
            )

        try:
            joint_data[joint_name] = float(
                position
            )

        except (
            TypeError,
            ValueError,
        ) as exception:
            raise ValueError(
                "관절값 변환 실패: "
                f"{joint_name}={position}"
            ) from exception

    return joint_data


def extract_joints(
    joint_data: dict[str, float],
    required_joint_names: list[str],
) -> list[float] | None:
    """
    필요한 관절이 모두 존재하면 지정한 순서대로 반환한다.

    해당 관절이 하나도 없으면 None을 반환하고,
    일부만 존재하면 오류로 처리한다.
    """

    existing_names = [
        joint_name
        for joint_name in required_joint_names
        if joint_name in joint_data
    ]

    if not existing_names:
        return None

    missing_names = [
        joint_name
        for joint_name in required_joint_names
        if joint_name not in joint_data
    ]

    if missing_names:
        raise ValueError(
            "필요한 관절 중 일부가 누락되었습니다. "
            f"존재={existing_names}, "
            f"누락={missing_names}"
        )

    return [
        joint_data[joint_name]
        for joint_name in required_joint_names
    ]


def radians_to_degrees(
    joint_positions_rad: list[float],
) -> list[float]:
    """MoveIt rad 값을 Doosan degree 값으로 변환한다."""

    return [
        math.degrees(position)
        for position in joint_positions_rad
    ]


def is_cartesian_planner(
    planner_id: str,
) -> bool:
    """
    planner 문자열에 CartesianPath가 포함됐는지 확인한다.
    """

    return (
        "cartesianpath"
        in planner_id.lower()
    )


def is_gripper_task(
    task_name: str,
) -> bool:
    """
    실제 그리퍼 동작 Stage인지 이름으로 판별한다.

    Connect Stage 등에 그리퍼 관절값이 함께 들어 있어도
    불필요한 재동작을 막기 위해 사용한다.
    """

    normalized_name = task_name.lower()

    keywords = [
        "gripper",
        "grasp",
        "release",
        "open",
        "close",
    ]

    return any(
        keyword in normalized_name
        for keyword in keywords
    )


def ensure_dsr_api_ready() -> None:
    """DSR API가 정상 바인딩되었는지 확인한다."""

    if movej is None:
        raise RuntimeError("movej가 초기화되지 않았습니다.")

    if movel is None:
        raise RuntimeError("movel이 초기화되지 않았습니다.")

    if fkin is None:
        raise RuntimeError("fkin이 초기화되지 않았습니다.")

    if posj is None:
        raise RuntimeError("posj가 초기화되지 않았습니다.")

    if gripper_cmd is None:
        raise RuntimeError(
            "gripper_cmd가 초기화되지 않았습니다."
        )

    if DR_BASE is None:
        raise RuntimeError("DR_BASE가 초기화되지 않았습니다.")


def execute_arm_trajectory(
    arm_positions_rad: list[float],
    planner_id: str,
) -> None:
    """
    CartesianPath면 fkin() 후 movel(),
    그 외 Planner면 movej()를 실행한다.
    """

    ensure_dsr_api_ready()

    if len(arm_positions_rad) != 6:
        raise ValueError(
            "로봇 팔 관절값은 정확히 6개여야 합니다. "
            f"현재 개수={len(arm_positions_rad)}"
        )

    arm_positions_deg = radians_to_degrees(
        arm_positions_rad
    )

    target_joint_position = posj(
        arm_positions_deg[0],
        arm_positions_deg[1],
        arm_positions_deg[2],
        arm_positions_deg[3],
        arm_positions_deg[4],
        arm_positions_deg[5],
    )

    if is_cartesian_planner(planner_id):
        target_task_position = fkin(
            target_joint_position,
            DR_BASE,
        )

        print(
            "[ARM][MOVEL] "
            f"joint_deg="
            f"{format_values(arm_positions_deg)}"
        )

        print(
            "[ARM][MOVEL] "
            f"target_task_position="
            f"{target_task_position}"
        )

        result = movel(
            target_task_position,
            vel=MOVEL_VELOCITY,
            acc=MOVEL_ACCELERATION,
        )

        if result != 0:
            raise RuntimeError(
                f"movel 실행 실패: return={result}"
            )

    else:
        print(
            "[ARM][MOVEJ] "
            f"planner={planner_id!r}, "
            f"joint_deg="
            f"{format_values(arm_positions_deg)}"
        )

        result = movej(
            target_joint_position,
            vel=MOVEJ_VELOCITY,
            acc=MOVEJ_ACCELERATION,
        )

        if result != 0:
            raise RuntimeError(
                f"movej 실행 실패: return={result}"
            )


def is_gripper_open(
    gripper_positions: list[float],
) -> bool:
    """
    네 개의 그리퍼 관절값이 모두 0에 가까우면
    열린 상태로 판단한다.
    """

    if len(gripper_positions) != 4:
        raise ValueError(
            "그리퍼 관절값은 정확히 4개여야 합니다. "
            f"현재 개수={len(gripper_positions)}"
        )

    return all(
        abs(position) <= GRIPPER_OPEN_THRESHOLD
        for position in gripper_positions
    )


def execute_gripper_trajectory(
    gripper_positions: list[float],
) -> None:
    """
    JSON의 그리퍼 상태를 열기/닫기 명령으로 변환한다.
    """

    ensure_dsr_api_ready()

    if is_gripper_open(gripper_positions):
        print(
            "[GRIPPER][OPEN] "
            f"trajectory="
            f"{format_values(gripper_positions)}"
        )

        result = gripper_cmd(
            GRIPPER_OPEN_POSITION,
            current=GRIPPER_OPEN_CURRENT,
        )

        if result is False:
            raise RuntimeError(
                f"그리퍼 열기 실패: return={result}"
            )

    else:
        print(
            "[GRIPPER][CLOSE] "
            f"trajectory="
            f"{format_values(gripper_positions)}"
        )

        result = gripper_cmd(
            GRIPPER_CLOSE_POSITION,
            current=GRIPPER_CLOSE_CURRENT,
        )

        if result is False:
            raise RuntimeError(
                f"그리퍼 닫기 실패: return={result}"
            )

    time.sleep(GRIPPER_WAIT_TIME)


def format_values(
    values: list[float],
    precision: int = 3,
) -> list[float]:
    """로그 출력을 위해 소수점 자릿수를 정리한다."""

    return [
        round(value, precision)
        for value in values
    ]


def execute_trajectory_entry(
    trajectory_entry: dict[str, Any],
    dry_run: bool,
) -> None:
    """JSON trajectory 항목 하나를 처리한다."""

    index = trajectory_entry.get(
        "index",
        "?",
    )

    stage_id = trajectory_entry.get(
        "stage_id",
        "?",
    )

    current_task = str(
        trajectory_entry.get(
            "current_task",
            "unknown",
        )
    )

    description = str(
        trajectory_entry.get(
            "description",
            "",
        )
    )

    planner_id = str(
        trajectory_entry.get(
            "planner",
            "",
        )
    )

    joint_data = parse_joint_data(
        trajectory_entry
    )

    arm_positions = extract_joints(
        joint_data,
        ARM_JOINT_NAMES,
    )

    gripper_positions = extract_joints(
        joint_data,
        GRIPPER_JOINT_NAMES,
    )

    print()
    print("=" * 72)

    print(
        f"[TRAJECTORY {index}] "
        f"stage_id={stage_id}"
    )

    print(f"Task        : {current_task}")
    print(f"Description : {description}")
    print(f"Planner     : {planner_id!r}")
    print(
        f"Joint names : "
        f"{list(joint_data.keys())}"
    )

    if (
        arm_positions is None
        and gripper_positions is None
    ):
        print(
            "[SKIP] 알려진 로봇 팔 또는 "
            "그리퍼 관절이 없습니다."
        )
        return

    if dry_run:
        if arm_positions is not None:
            move_type = (
                "MOVEL"
                if is_cartesian_planner(planner_id)
                else "MOVEJ"
            )

            print(
                f"[DRY RUN][ARM][{move_type}] "
                f"joint_rad="
                f"{format_values(arm_positions)}, "
                f"joint_deg="
                f"{format_values(radians_to_degrees(arm_positions))}"
            )

        if (
            gripper_positions is not None
            and is_gripper_task(current_task)
        ):
            gripper_action = (
                "OPEN"
                if is_gripper_open(gripper_positions)
                else "CLOSE"
            )

            print(
                f"[DRY RUN][GRIPPER]"
                f"[{gripper_action}] "
                f"values="
                f"{format_values(gripper_positions)}"
            )

        elif gripper_positions is not None:
            print(
                "[DRY RUN][GRIPPER][SKIP] "
                "그리퍼 관절값은 포함되어 있지만 "
                "그리퍼 동작 Stage가 아닙니다."
            )

        return

    if arm_positions is not None:
        execute_arm_trajectory(
            arm_positions,
            planner_id,
        )

    if (
        gripper_positions is not None
        and is_gripper_task(current_task)
    ):
        execute_gripper_trajectory(
            gripper_positions,
        )


def execute_json_file(
    json_path: Path,
    dry_run: bool,
    start_index: int,
) -> None:
    """JSON trajectory 배열을 index 순서로 실행한다."""

    data = load_trajectory_json(
        json_path
    )

    scene_number = data.get(
        "scene_number",
        "unknown",
    )

    created_at = data.get(
        "created_at",
        "unknown",
    )

    trajectories = data["trajectories"]

    sorted_trajectories = sorted(
        trajectories,
        key=lambda entry: int(
            entry.get(
                "index",
                sys.maxsize,
            )
        ),
    )

    print(f"JSON file        : {json_path}")
    print(f"Scene number     : {scene_number}")
    print(f"Created at       : {created_at}")

    print(
        f"Trajectory count : "
        f"{len(sorted_trajectories)}"
    )

    print(f"Start index      : {start_index}")
    print(f"Dry run          : {dry_run}")

    for trajectory_entry in sorted_trajectories:
        trajectory_index = int(
            trajectory_entry.get(
                "index",
                0,
            )
        )

        if trajectory_index < start_index:
            continue

        execute_trajectory_entry(
            trajectory_entry,
            dry_run,
        )

    print()
    print("모든 trajectory 처리가 완료되었습니다.")


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "doosan_mini_project_moveit_config 패키지의 "
            "고정된 MTC trajectory JSON을 읽어 "
            "Doosan 로봇과 그리퍼를 순서대로 실행합니다."
        )
    )

    parser.add_argument(
        "--execute",
        action="store_true",
        help=(
            "실제 로봇 명령을 실행합니다. "
            "지정하지 않으면 dry-run입니다."
        ),
    )

    parser.add_argument(
        "--start-index",
        type=int,
        default=1,
        help=(
            "실행을 시작할 trajectory index "
            "(기본값: 1)"
        ),
    )

    return parser.parse_args()


def initialize_dsr_api(node: Any) -> Any:
    """
    DR_init 설정 후 DSR_ROBOT2를 import하고,
    전역 DSR API 변수에 연결한다.
    """

    global movej
    global movel
    global fkin
    global posj
    global gripper_cmd
    global DR_BASE

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import (
        DR_BASE as dsr_base,
        ROBOT_MODE_AUTONOMOUS,
        fkin as dsr_fkin,
        movej as dsr_movej,
        movel as dsr_movel,
        posj as dsr_posj,
        set_robot_mode,
    )

    from dsr_gripper import (
        gripper_cmd as dsr_gripper_cmd,
    )

    DR_BASE = dsr_base
    fkin = dsr_fkin
    gripper_cmd = dsr_gripper_cmd
    movej = dsr_movej
    movel = dsr_movel
    posj = dsr_posj

    return (
        set_robot_mode,
        ROBOT_MODE_AUTONOMOUS,
    )


def main() -> int:
    args = parse_arguments()

    if args.start_index < 1:
        print(
            "--start-index는 1 이상이어야 합니다.",
            file=sys.stderr,
        )
        return 1

    node = None

    try:
        # 1. rclpy 초기화
        rclpy.init()

        # 2. 반드시 dsr01 namespace로 노드 생성
        node = rclpy.create_node(
            "doosan_json_trajectory_executor",
            namespace=ROBOT_ID,
        )

        # 3. DR_init에 노드를 등록한 뒤 DSR_ROBOT2 import
        (
            set_robot_mode,
            robot_mode_autonomous,
        ) = initialize_dsr_api(node)

        if args.execute:
            set_robot_mode(
                robot_mode_autonomous
            )

            print("로봇이 연결되었습니다.")

        else:
            print(
                "Dry-run 모드입니다. "
                "실제 로봇 명령은 실행하지 않습니다."
            )

        json_path = get_fixed_json_path()

        execute_json_file(
            json_path=json_path,
            dry_run=not args.execute,
            start_index=args.start_index,
        )

    except KeyboardInterrupt:
        print()
        print("사용자에 의해 실행이 중단되었습니다.")
        return 130

    except Exception as exception:
        print(
            f"실행 실패: {exception}",
            file=sys.stderr,
        )
        return 1

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())