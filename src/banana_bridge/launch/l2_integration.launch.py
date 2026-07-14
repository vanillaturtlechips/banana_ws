"""L2 통합 테스트 런치 — 카메라 + perception + 브리지를 한 번에.

채팅 → LLM → SortCommand → perception(실카메라 YOLO) → /banana/detection
   → 브리지가 직접 구독 → LiveState(웹) . 로봇은 스텁(aggregator/MoveIt 없음).

사용 (⚠️ YOLO 의존이 있는 venv 활성화 후):
  source /root/venv/yolov3/bin/activate
  source install/setup.bash
  ros2 launch banana_bridge l2_integration.launch.py
인자:
  use_camera:=true      RealSense 드라이버 실행 (다른 카메라 소스면 false)
  bridge_port:=8000
  llm_provider:=fake    fake(키불필요) | api | local

perception 노드는 ROS 파이썬(/usr/bin/python3)으로 뜨는데 거기엔 torch/ultralytics가
없다. 그래서 활성 venv(VIRTUAL_ENV)의 site-packages를 PYTHONPATH에 얹어 실 YOLO를 켠다.
(없으면 StubDetector 폴백)
"""
import glob
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            ExecuteProcess, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_camera = LaunchConfiguration("use_camera")
    bridge_port = LaunchConfiguration("bridge_port")
    llm_provider = LaunchConfiguration("llm_provider")

    actions = [
        DeclareLaunchArgument("use_camera", default_value="true",
                              description="RealSense 드라이버 실행"),
        DeclareLaunchArgument("bridge_port", default_value="8000"),
        DeclareLaunchArgument("llm_provider", default_value="fake",
                              description="fake | api | local"),
    ]

    # 활성 venv의 site-packages를 PYTHONPATH에 얹음 → perception이 torch/ultralytics import 가능
    venv = os.environ.get("VIRTUAL_ENV")
    if venv:
        sp = glob.glob(os.path.join(venv, "lib", "python*", "site-packages"))
        if sp:
            actions.append(SetEnvironmentVariable(
                "PYTHONPATH", sp[0] + os.pathsep + os.environ.get("PYTHONPATH", "")))

    actions += [
        # ① RealSense (컬러 + 정렬 뎁스 + camera_info). infra 끔(IR 경고 방지)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"])),
            launch_arguments={"align_depth.enable": "true",
                              "enable_infra": "false",
                              "enable_infra1": "false",
                              "enable_infra2": "false"}.items(),
            condition=IfCondition(use_camera),
        ),

        # ② perception (실카메라 YOLO → /banana/detection)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("banana_perception"), "launch", "perception.launch.py"])),
            launch_arguments={"use_fake_camera": "false"}.items(),
        ),

        # ③ 브리지 (uvicorn, ros 백엔드 = 우리 상태-IN 가닥)
        ExecuteProcess(
            cmd=["uvicorn", "banana_bridge.app:app",
                 "--host", "0.0.0.0", "--port", bridge_port],
            additional_env={"BANANA_FAKE": "0", "BANANA_LLM_PROVIDER": llm_provider},
            output="screen",
        ),
    ]
    return LaunchDescription(actions)
