"""perception 노드 실행.

인자:
  use_fake_camera:=true    합성 카메라도 같이 띄움
  publish_handeye:=true    hand-eye 정적 TF(base_link→카메라) 발행 (아래 값은 ⚠️placeholder)

⚠️ hand-eye 캘리브:
  아래 static_transform_publisher 의 x/y/z/roll/pitch/yaw 는 **예시값**입니다.
  실제 로봇에서 easy_handeye 등으로 캘리브한 값으로 반드시 교체하세요.
  (실제 카메라 드라이버가 이 TF를 직접 발행한다면 publish_handeye:=false 로 끄면 됨)
"""
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
    pub_handeye = LaunchConfiguration("publish_handeye")

    return LaunchDescription([
        DeclareLaunchArgument("use_fake_camera", default_value="true",
                              description="합성 카메라 노드도 실행"),
        DeclareLaunchArgument("publish_handeye", default_value="true",
                              description="hand-eye 정적 TF 발행 (placeholder 값)"),

        Node(package="banana_perception", executable="perception",
             parameters=[cfg], output="screen"),
        Node(package="banana_perception", executable="fake_camera",
             condition=IfCondition(use_fake), output="screen"),

        # ✅ 실측 hand-eye: base_link → camera_color_optical_frame
        #    PinkLab HandEyeCal (eye-to-hand, TSAI, 12샘플, verify std max 9.94mm=GOOD, 2026-07-14)
        #    카메라 위치 base 기준 (0.839, -0.014, 0.926)m, yaw≈91°(광축 아래). 원본: 결과/T_base_camera.yaml
        #    쿼터니언으로 넣어 회전 정밀 유지(RPY 변환 오차 방지).
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="handeye_static_tf",
             condition=IfCondition(pub_handeye),
             arguments=[
                 "--x", "0.839394", "--y", "-0.014461", "--z", "0.925965",
                 "--qx", "-0.700741", "--qy", "-0.712613",
                 "--qz", "-0.016235", "--qw", "0.029682",
                 "--frame-id", "base_link",
                 # RealSense가 카메라 내부 TF를 발행 안 하므로 optical을 직접 연결한다.
                 # (RealSense publish_tf가 켜져 camera_link→optical이 있으면 camera_link로 바꿀 것)
                 "--child-frame-id", "camera_color_optical_frame",
             ],
             output="screen"),
    ])
