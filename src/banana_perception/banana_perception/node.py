"""Perception 노드 — 카메라 프레임 → YOLO 추론 → Detection 발행.

  IN  : image (카메라), SortCommand (브리지)
  OUT : Detection (감지 결과 → aggregator/bridge)

모델 없으면 StubDetector로 폴백 → GPU/가중치 없이도 노드가 뜬다.
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from banana_command.msg import SortCommand, Detection

from .detector import make_detector, STAGES
from .filter import select_target

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST, depth=1,
)


class PerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("banana_perception")

        # --- 파라미터 (launch/yaml에서 덮어씀) ---
        p = self.declare_parameter
        self._image_topic = p("image_topic", "/camera/color/image_raw").value
        self._cmd_topic = p("command_topic", "/banana/command").value
        self._det_topic = p("detection_topic", "/banana/detection").value
        self._model_path = p("model_path", "models/best.pt").value
        self._conf = p("conf_threshold", 0.5).value
        self._classes = p("classes", list(STAGES)).value
        self._device = p("device", "cuda:0").value

        # --- 감지기 (모델 없으면 Stub) ---
        self._detector = make_detector(
            self._model_path, self._classes, self._conf, self._device)

        # --- 명령 상태 (target_stages 비었으면 전체) ---
        self._target_stages: list[str] = []

        # --- I/O ---
        self._pub = self.create_publisher(Detection, self._det_topic, 10)
        self.create_subscription(Image, self._image_topic, self._on_image, SENSOR_QOS)
        self.create_subscription(SortCommand, self._cmd_topic, self._on_command, 10)

        self.get_logger().info(
            f"perception 시작: {self._image_topic} → {self._det_topic} "
            f"(detector={type(self._detector).__name__})")

    # ---- IN: 명령 ----
    def _on_command(self, msg: SortCommand) -> None:
        if msg.action == "sort":
            self._target_stages = list(msg.target_stages)
            self.get_logger().info(f"명령: sort {self._target_stages or '전체'}")
        elif msg.action in ("stop", "rescan"):
            self._target_stages = []

    # ---- IN: 이미지 → 추론 → 발행 ----
    def _on_image(self, msg: Image) -> None:
        frame = self._to_numpy(msg)
        dets = self._detector.detect(frame)
        target = select_target(dets, self._target_stages)
        if target is not None:
            self._pub.publish(self._to_msg(target, msg.header))

    @staticmethod
    def _to_numpy(msg: Image) -> np.ndarray:
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == "bgr8":
            arr = arr[:, :, ::-1]
        return np.ascontiguousarray(arr[:, :, :3])

    def _to_msg(self, d, header) -> Detection:  # noqa: ANN001
        m = Detection()
        m.header = header
        m.stage = d.stage
        m.confidence = float(d.confidence)
        m.x, m.y, m.w, m.h = int(d.x), int(d.y), int(d.w), int(d.h)
        m.image_width, m.image_height = int(d.image_width), int(d.image_height)
        return m


def main() -> None:
    rclpy.init()
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
