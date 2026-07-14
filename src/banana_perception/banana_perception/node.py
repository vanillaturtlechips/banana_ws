"""Perception 노드 — 카메라 프레임 → YOLO 추론 → Detection 발행.

  IN  : color image, aligned depth, camera_info, SortCommand
  OUT : Detection (2D bbox+회전 + 3D deproject + base_link grasp_pose)

3D 파이프라인:
  픽셀(u,v) --뎁스--> Z(m) --intrinsics--> 카메라프레임 XYZ --TF(hand-eye)--> base_link pose

뎁스/CameraInfo/TF 중 없는 단계는 graceful 폴백(has_depth/has_pose=false)해서
모델·GPU·캘리브 없이도 노드는 계속 뜬다.
"""
from __future__ import annotations

import dataclasses
import math
from collections import Counter, deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from banana_command.msg import SortCommand, Detection, DetectionArray

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (PointStamped do_transform 등록용)

from .detector import make_detector, STAGES
from .filter import select_target

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST, depth=1,
)


class PerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("banana_perception")

        p = self.declare_parameter
        self._image_topic = p("image_topic", "/camera/color/image_raw").value
        self._depth_topic = p("depth_topic",
                              "/camera/aligned_depth_to_color/image_raw").value
        self._caminfo_topic = p("camera_info_topic", "/camera/color/camera_info").value
        self._cmd_topic = p("command_topic", "/banana/command").value
        self._det_topic = p("detection_topic", "/banana/detection").value
        self._dets_topic = p("detections_topic", "/banana/detections").value
        self._model_path = p("model_path", "models/best.pt").value
        self._conf = p("conf_threshold", 0.5).value
        self._classes = p("classes", list(STAGES)).value
        self._device = p("device", "cuda:0").value
        self._base_frame = p("base_frame", "base_link").value
        self._depth_scale = p("depth_scale", 0.001).value   # 16UC1 mm → m
        # 시간 스무딩: 최근 N프레임 다수결 클래스로 깜빡임 억제
        self._vote = deque(maxlen=p("vote_window", 12).value)

        self._detector = make_detector(
            self._model_path, self._classes, self._conf, self._device)
        self._target_stages: list[str] = []

        # 3D 상태
        self._depth: np.ndarray | None = None     # 최신 정렬 뎁스 (uint16, mm)
        self._K: tuple | None = None              # (fx, fy, cx, cy)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # I/O
        self._pub = self.create_publisher(Detection, self._det_topic, 10)
        self._arr_pub = self.create_publisher(DetectionArray, self._dets_topic, 10)
        self.create_subscription(Image, self._image_topic, self._on_image, SENSOR_QOS)
        self.create_subscription(Image, self._depth_topic, self._on_depth, SENSOR_QOS)
        self.create_subscription(CameraInfo, self._caminfo_topic, self._on_caminfo, SENSOR_QOS)
        self.create_subscription(SortCommand, self._cmd_topic, self._on_command, 10)

        self.get_logger().info(
            f"perception 시작: {self._image_topic}(+depth) → {self._det_topic} "
            f"(detector={type(self._detector).__name__}, base={self._base_frame})")

    # ---- IN: 명령 ----
    def _on_command(self, msg: SortCommand) -> None:
        if msg.action == "sort":
            self._target_stages = list(msg.target_stages)
            self.get_logger().info(f"명령: sort {self._target_stages or '전체'}")
        elif msg.action in ("stop", "rescan"):
            self._target_stages = []

    # ---- IN: 뎁스 / CameraInfo ----
    def _on_depth(self, msg: Image) -> None:
        self._depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)

    def _on_caminfo(self, msg: CameraInfo) -> None:
        k = msg.k
        self._K = (k[0], k[4], k[2], k[5])   # fx, fy, cx, cy

    # ---- IN: 이미지 → 추론 → 발행 ----
    def _on_image(self, msg: Image) -> None:
        frame = self._to_numpy(msg)
        dets = self._detector.detect(frame)

        # 멀티 오브젝트: 감지 전부를 배열로 발행 (셀렉터/게이트/멀티다이스용)
        arr = DetectionArray()
        arr.header = msg.header
        arr.detections = [self._to_msg(d, msg.header) for d in dets]
        self._arr_pub.publish(arr)

        # 단일 최적 1개 (기존 웹/브리지 호환, voting 적용)
        target = select_target(dets, self._target_stages)
        # temporal voting: 최근 창 다수결 클래스로 스무딩 (인접클래스 깜빡임 억제)
        self._vote.append(target.stage if target is not None else None)
        if target is not None:
            votes = [v for v in self._vote if v is not None]
            voted = Counter(votes).most_common(1)[0][0] if votes else target.stage
            if voted != target.stage:
                alt = [d for d in dets if d.stage == voted]
                target = (max(alt, key=lambda d: d.confidence) if alt
                          else dataclasses.replace(
                              target, stage=voted,
                              class_id=self._classes.index(voted)))
            self._pub.publish(self._to_msg(target, msg.header))

    @staticmethod
    def _to_numpy(msg: Image) -> np.ndarray:
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == "bgr8":
            arr = arr[:, :, ::-1]
        return np.ascontiguousarray(arr[:, :, :3])

    def _depth_at(self, u: float, v: float, win: int = 5) -> float:
        """중심 픽셀 근방 유효 뎁스 중앙값(m). 없으면 0.0."""
        d = self._depth
        if d is None:
            return 0.0
        h, w = d.shape
        u, v = int(round(u)), int(round(v))
        if not (0 <= u < w and 0 <= v < h):
            return 0.0
        roi = d[max(0, v - win):v + win + 1, max(0, u - win):u + win + 1]
        vals = roi[roi > 0]
        return float(np.median(vals)) * self._depth_scale if vals.size else 0.0

    def _to_msg(self, d, header) -> Detection:  # noqa: ANN001
        m = Detection()
        m.header = header
        m.stage = d.stage
        m.class_id = int(d.class_id)
        m.confidence = float(d.confidence)
        m.x, m.y, m.w, m.h = int(d.x), int(d.y), int(d.w), int(d.h)
        m.image_width, m.image_height = int(d.image_width), int(d.image_height)
        m.center_x, m.center_y = float(d.center_x), float(d.center_y)
        m.rect_w, m.rect_h = float(d.rect_w), float(d.rect_h)
        m.angle_deg = float(d.angle_deg)

        # --- 3D: deproject ---
        z = self._depth_at(d.center_x, d.center_y)
        if z > 0.0 and self._K is not None:
            fx, fy, cx, cy = self._K
            X = (d.center_x - cx) * z / fx
            Y = (d.center_y - cy) * z / fy
            m.has_depth = True
            m.center_z = float(z)
            m.point_x, m.point_y, m.point_z = float(X), float(Y), float(z)
            # --- base_link 변환 (hand-eye TF) ---
            self._fill_pose(m, header, X, Y, z, d.angle_deg)
        return m

    def _fill_pose(self, m: Detection, header, X, Y, Z, angle_deg) -> None:
        try:
            pt = PointStamped()
            pt.header.frame_id = header.frame_id
            pt.header.stamp = header.stamp
            pt.point.x, pt.point.y, pt.point.z = float(X), float(Y), float(Z)
            tf = self._tf_buffer.lookup_transform(
                self._base_frame, header.frame_id, rclpy.time.Time())
            pb = tf2_geometry_msgs.do_transform_point(pt, tf)
            m.grasp_pose.header.frame_id = self._base_frame
            m.grasp_pose.header.stamp = header.stamp
            m.grasp_pose.pose.position = pb.point
            yaw = math.radians(angle_deg)   # 이미지평면 회전 → base yaw (탑다운 근사)
            m.grasp_pose.pose.orientation.z = math.sin(yaw / 2.0)
            m.grasp_pose.pose.orientation.w = math.cos(yaw / 2.0)
            m.has_pose = True
        except Exception:      # TF 없음/캘리브 전 → 폴백
            m.has_pose = False


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
