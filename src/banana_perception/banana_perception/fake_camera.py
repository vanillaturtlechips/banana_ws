"""가짜 카메라 노드 — 실물 카메라 없이 합성 프레임 발행.

perception 노드 테스트용. 실제 카메라 드라이버(realsense/orbbec)로 교체하면 됨.
"""
from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


class FakeCamera(Node):
    def __init__(self) -> None:
        super().__init__("fake_camera")
        self._topic = self.declare_parameter(
            "image_topic", "/camera/color/image_raw").value
        self._w = self.declare_parameter("width", 640).value
        self._h = self.declare_parameter("height", 480).value
        fps = self.declare_parameter("fps", 15).value

        self._pub = self.create_publisher(Image, self._topic, 10)
        self._t0 = time.time()
        self.create_timer(1.0 / fps, self._tick)
        self.get_logger().info(f"fake_camera → {self._topic} ({self._w}x{self._h})")

    def _tick(self) -> None:
        t = time.time() - self._t0
        xs = np.linspace(0, 1, self._w)
        shift = (math.sin(t) + 1) / 2
        r = (xs + shift) % 1.0
        row = np.stack([r, np.full(self._w, 0.85), np.full(self._w, 0.2)], axis=1)
        frame = (np.tile(row, (self._h, 1, 1)) * 255).astype(np.uint8)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.height, msg.width = self._h, self._w
        msg.encoding = "rgb8"
        msg.step = self._w * 3
        msg.data = frame.tobytes()
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = FakeCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
