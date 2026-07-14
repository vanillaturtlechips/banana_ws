"""실물 rclpy 백엔드 — FakeBridge와 동일 인터페이스.

app.py는 이 클래스와 FakeBridge를 같은 방식으로 다룬다:
  start() / stop() / publish_command(model) / state_stream()
rclpy는 별 스레드에서 spin. rclpy import는 start() 안에서 (페이크 모드가 ROS 없이 돌게).
"""
from __future__ import annotations

import asyncio
import os
import threading
from typing import AsyncIterator, Optional

import numpy as np

from ..domain.schema import SortCommandModel
from ..domain.state import to_live_state, _map_detection_msg
from ..transport.webrtc import FrameSource

_BIN_LABEL = {"bin1": "1번 보관함", "bin2": "2번 보관함(쓰레기통)"}


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
        # 병합 상태(미니 aggregator): 감지 + 에이전트 상태를 한 LiveState로
        self._live: dict = {
            "robot": "idle", "robotMessage": "바나나를 기다리는 중...",
            "detection": None, "logs": [], "exceptions": [],
        }

        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
        from banana_command.msg import SortCommand, Detection

        self._SortCommand = SortCommand
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )

        rclpy.init()
        self._node = Node("banana_bridge")
        self._cmd_pub = self._node.create_publisher(SortCommand, "/banana/command", 10)

        # aggregator 우회(L2): 감지 토픽 직접 구독 → LiveState 로 웹에 push.
        # aggregator(/banana/system_state) 생기면 아래를 그쪽 구독으로 교체.
        self._node.create_subscription(
            Detection, "/banana/detection", self._on_detection, 10)
        self._node.create_subscription(
            String, "/banana/agent_status", self._on_agent_status, 10)
        cam_topic = os.getenv("BANANA_CAMERA_TOPIC", "/camera/camera/color/image_raw")
        self._node.create_subscription(Image, cam_topic, self._on_image, sensor_qos)

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

    def _push(self) -> None:
        self._loop.call_soon_threadsafe(self._q.put_nowait, dict(self._live))

    def _on_detection(self, msg) -> None:  # noqa: ANN001
        # 감지 부분만 갱신 (robot 상태는 에이전트가 갱신)
        self._live["detection"] = _map_detection_msg(msg)
        self._push()

    def _on_agent_status(self, msg) -> None:  # noqa: ANN001
        # 에이전트 결정 → robot/robotMessage 갱신 (선택·게이트 결과)
        import json
        try:
            st = json.loads(msg.data)
        except Exception:
            return
        if st.get("action") in ("stop", "rescan"):
            self._live["robot"] = "idle"
            self._live["robotMessage"] = "대기 중이에요"
        elif st.get("ok"):
            dest = _BIN_LABEL.get(st.get("bin"), st.get("bin"))
            self._live["robot"] = "picking"
            self._live["robotMessage"] = f"{st.get('stage')} → {dest}(으)로 집는 중 🦾"
        else:  # 게이트 차단 (unripe 등)
            self._live["robot"] = "error"
            self._live["robotMessage"] = st.get("reason") or "그 명령은 못 해요"
        self._push()

    def _on_image(self, msg) -> None:  # noqa: ANN001
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == "bgr8":
            arr = arr[:, :, ::-1]
        self._fs.put(np.ascontiguousarray(arr[:, :, :3]))

    # ---- IN: asyncio 소비 ----------------------------------------
    async def state_stream(self) -> AsyncIterator[dict]:
        while True:
            yield await self._q.get()
