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

        # ⚠️ placeholder hand-eye: base_link → camera_color_optical_frame
        #    (탑다운 예시: 카메라가 base 위 z=0.9m, 아래를 봄 → roll=pi)
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="handeye_static_tf",
             condition=IfCondition(pub_handeye),
             arguments=[
                 "--x", "0.40", "--y", "0.0", "--z", "0.90",
                 "--roll", "3.14159", "--pitch", "0.0", "--yaw", "0.0",
                 "--frame-id", "base_link",
                 # 카메라 루트(camera_link)로 연결 — optical은 RealSense가 발행하므로
                 # 여기서 optical을 직접 가리키면 부모 충돌. base_link→camera_link→…→optical.
                 "--child-frame-id", "camera_link",
             ],
             output="screen"),
    ])
