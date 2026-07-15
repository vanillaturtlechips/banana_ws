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
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_camera = LaunchConfiguration("use_camera")
    bridge_port = LaunchConfiguration("bridge_port")
    llm_provider = LaunchConfiguration("llm_provider")
    llm_model = LaunchConfiguration("llm_model")

    actions = [
        DeclareLaunchArgument("use_camera", default_value="true",
                              description="RealSense 드라이버 실행"),
        DeclareLaunchArgument("bridge_port", default_value="8000"),
        DeclareLaunchArgument("llm_provider", default_value="fake",
                              description="fake | api | local"),
        # LLM 모델: 활성은 qwen2.5:7b. 아래 3개는 벤치마크용 (주석):
        #   nova2
        #   ollama
        #   haiku
        DeclareLaunchArgument("llm_model", default_value="qwen2.5:7b",
                              description="LLM 모델명 (활성: qwen2.5:7b, 나머지는 벤치마크용)"),
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
                              # 학습 촬영과 동일 해상도(1280x800)로 맞춰 색·크기 도메인시프트 완화
                              "rgb_camera.color_profile": "1280x800x30",
                              "initial_reset": "true",   # wedge된 장치 기동 시 리셋
                              # ⚠️ RealSense 자체 카메라 TF 끔. 안 끄면 camera_color_optical_frame이
                              #    RealSense(camera_color_frame 자식) + hand-eye(base_link 자식)로
                              #    부모가 둘 → TF 트리 분리. hand-eye가 optical을 단일 발행하게 함.
                              "publish_tf": "false",
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

        # ③ 에이전트 (command + detections → 선택+게이트 → pick_target + status)
        Node(package="banana_perception", executable="agent", output="screen"),

        # ④ 브리지 (uvicorn, ros 백엔드 = 우리 상태-IN 가닥)
        ExecuteProcess(
            cmd=["uvicorn", "banana_bridge.app:app",
                 "--host", "0.0.0.0", "--port", bridge_port],
            additional_env={"BANANA_FAKE": "0",
                            "BANANA_LLM_PROVIDER": llm_provider,
                            "BANANA_LLM_LOCAL_MODEL": llm_model},
            output="screen",
        ),
    ]
    return LaunchDescription(actions)
