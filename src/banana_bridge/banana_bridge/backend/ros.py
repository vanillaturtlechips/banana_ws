"""실물 rclpy 백엔드 — FakeBridge와 동일 인터페이스.

app.py는 이 클래스와 FakeBridge를 같은 방식으로 다룬다:
  start() / stop() / publish_command(model) / state_stream()
rclpy는 별 스레드에서 spin. rclpy import는 start() 안에서 (페이크 모드가 ROS 없이 돌게).
"""
from __future__ import annotations

import asyncio
import threading
from typing import AsyncIterator, Optional

import numpy as np

from ..domain.schema import SortCommandModel
from ..domain.state import to_live_state
from ..transport.webrtc import FrameSource


class BridgeRos:
    def __init__(self, frame_source: FrameSource) -> None:
        self._fs = frame_source
        self._node = None
        self._cmd_pub = None
        self._thread: Optional[threading.Thread] = None
        # 루프/큐는 start()에서 잡는다. __init__은 모듈 import 시점(서버 루프 전)이라
        # 여기서 get_event_loop()하면 엉뚱한 루프를 잡아 상태가 유실됨.
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._q: Optional["asyncio.Queue[dict]"] = None

    # ---- 수명주기 -------------------------------------------------
    def start(self) -> None:
        # start()는 _startup(async, 서버 루프 안)에서 호출 → 올바른 루프를 잡음.
        self._loop = asyncio.get_running_loop()
        self._q = asyncio.Queue()

        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import Image
        from banana_command.msg import SortCommand

        self._SortCommand = SortCommand
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )

        rclpy.init()
        self._node = Node("banana_bridge")
        self._cmd_pub = self._node.create_publisher(SortCommand, "/banana/command", 10)

        # TODO: SystemState 타입/토픽/QoS는 aggregator 담당자와 합의
        # from your_state_pkg.msg import SystemState
        # self._node.create_subscription(SystemState, "/banana/system_state",
        #                                self._on_state, 10)
        self._node.create_subscription(
            Image, "/camera/color/image_raw", self._on_image, sensor_qos)

        self._thread = threading.Thread(
            target=lambda: rclpy.spin(self._node), daemon=True)
        self._thread.start()

    def stop(self) -> None:
        import rclpy
        if self._node is not None:
            self._node.destroy_node()
        rclpy.shutdown()

    # ---- OUT ------------------------------------------------------
    def publish_command(self, model: SortCommandModel) -> None:
        if self._cmd_pub is None:
            return
        import json
        msg = self._SortCommand()
        msg.action = model.action.value
        msg.target_stages = [s.value for s in model.target_stages]
        msg.params_json = json.dumps(model.params)
        self._cmd_pub.publish(msg)

    # ---- IN: 콜백 (rclpy 스레드) → asyncio 큐 --------------------
    def _on_state(self, msg) -> None:  # noqa: ANN001
        live = to_live_state(msg)
        self._loop.call_soon_threadsafe(self._q.put_nowait, live)  # 스레드→asyncio

    def _on_image(self, msg) -> None:  # noqa: ANN001
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == "bgr8":
            arr = arr[:, :, ::-1]
        self._fs.put(np.ascontiguousarray(arr[:, :, :3]))

    # ---- IN: asyncio 소비 ----------------------------------------
    async def state_stream(self) -> AsyncIterator[dict]:
        while True:
            yield await self._q.get()
