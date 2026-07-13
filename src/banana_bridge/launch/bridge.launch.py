"""브리지 실행 launch. uvicorn을 프로세스로 띄운다 (rclpy spin은 app 내부 스레드)."""
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                "uvicorn", "banana_bridge.app:app",
                "--host", "0.0.0.0", "--port", "8000",
            ],
            output="screen",
        ),
    ])
