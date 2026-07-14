"""조립 루트 (composition root).

배관(transport/) + 백엔드(backend/) + 도메인(domain/)을 엮기만 한다.
실제 로직은 각 모듈에. 여기는 "무엇을 어떻게 연결하는가"만 보이게.

  BANANA_FAKE=1 (기본) → FakeBridge (ROS/하드웨어 불필요, 지금 바로 실행)
  BANANA_FAKE=0        → BridgeRos  (실물 rclpy)

실행: BANANA_FAKE=1 uvicorn banana_bridge.app:app --port 8000
"""
from __future__ import annotations

import asyncio
import os
from contextlib import asynccontextmanager

from starlette.applications import Starlette
from starlette.routing import WebSocketRoute
from starlette.websockets import WebSocket

from .transport.webrtc import FrameSource, WebRTCManager
from .transport.dispatch import Dispatcher
from .transport.gateway import Broadcaster, make_gateway
from .backend.base import Backend
from .domain.llm import parse_utterance

# --- 구성 요소 -----------------------------------------------------
frame_source = FrameSource()
webrtc = WebRTCManager(frame_source)

if os.getenv("BANANA_FAKE", "1") == "1":
    from .backend.fake import FakeBridge
    backend: Backend = FakeBridge(frame_source)
else:
    from .backend.ros import BridgeRos
    backend = BridgeRos(frame_source)

broadcaster = Broadcaster()
bus = Dispatcher()

# --- 도메인 핸들러 등록 (IN 메시지) --------------------------------
@bus.on("chat")
async def _chat(ws: WebSocket, data: dict) -> None:
    model, reply = await parse_utterance(data["payload"])
    if model is not None:
        backend.publish_command(model)
    await ws.send_json({"type": "chat", "payload": reply})


@bus.on("webrtc/offer")
async def _offer(ws: WebSocket, data: dict) -> None:
    try:
        answer = await webrtc.handle_offer(
            ws.state.cid, data["sdp"], data.get("sdpType", "offer"))
        await ws.send_json(answer)
    except Exception:   # 새로고침/종료로 클라가 사라지면 answer 전송 실패 → 정리만
        await webrtc.close(ws.state.cid)


@bus.on("webrtc/bye")
async def _bye(ws: WebSocket, data: dict) -> None:
    await webrtc.close(ws.state.cid)


async def _cleanup(ws: WebSocket) -> None:
    await webrtc.close(ws.state.cid)


# --- 수명주기 ------------------------------------------------------
async def _startup() -> None:
    backend.start()
    asyncio.create_task(
        broadcaster.pump(
            backend.state_stream(),
            lambda live: {"type": "state", "payload": live},  # OUT: 상태 방송
        )
    )


async def _shutdown() -> None:
    backend.stop()
    await webrtc.close_all()


@asynccontextmanager
async def _lifespan(_app: Starlette):
    await _startup()
    try:
        yield
    finally:
        await _shutdown()


app = Starlette(
    routes=[WebSocketRoute("/ws", make_gateway(bus, broadcaster, on_cleanup=_cleanup))],
    lifespan=_lifespan,   # 최신 Starlette: on_startup/on_shutdown 대신 lifespan
)
