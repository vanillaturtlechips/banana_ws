"""WebSocket 게이트웨이 배관 (도메인 무관, 재사용).

  - Broadcaster : 구독자 집합 관리 + 안전한 방송(죽은 연결 격리) + 스트림 펌프
  - make_gateway: 연결 수명주기를 처리하는 WebSocketEndpoint 생성
                  (on_connect/on_receive/on_disconnect → try/except 불필요)

도메인(바나나) 로직은 여기 없음. dispatcher/broadcaster/cleanup만 주입받는다.
"""
from __future__ import annotations

import asyncio
import uuid
from typing import AsyncIterator, Awaitable, Callable, Optional

from starlette.endpoints import WebSocketEndpoint
from starlette.websockets import WebSocket

from .dispatch import Dispatcher


class Broadcaster:
    """여러 WS 연결에 동시 방송. 실패한 연결은 걸러낸다."""

    def __init__(self) -> None:
        self._subs: set[WebSocket] = set()

    def add(self, ws: WebSocket) -> None:
        self._subs.add(ws)

    def discard(self, ws: WebSocket) -> None:
        self._subs.discard(ws)

    async def broadcast(self, message: dict) -> None:
        targets = list(self._subs)
        if not targets:
            return
        results = await asyncio.gather(
            *(ws.send_json(message) for ws in targets),
            return_exceptions=True,  # 예외를 값으로 받아 죽은 연결만 제거
        )
        for ws, res in zip(targets, results):
            if isinstance(res, Exception):
                self._subs.discard(ws)

    async def pump(self, source: AsyncIterator, wrap: Callable[[object], dict]) -> None:
        """비동기 소스를 구독자 전체로 흘린다. wrap으로 메시지 포장."""
        async for item in source:
            await self.broadcast(wrap(item))


def make_gateway(
    dispatcher: Dispatcher,
    broadcaster: Broadcaster,
    on_cleanup: Optional[Callable[[WebSocket], Awaitable[None]]] = None,
) -> type[WebSocketEndpoint]:
    """의존성을 주입해 WebSocketEndpoint 서브클래스를 생성."""

    class Gateway(WebSocketEndpoint):
        encoding = "json"

        async def on_connect(self, ws: WebSocket) -> None:
            await ws.accept()
            ws.state.cid = uuid.uuid4().hex  # 연결에 귀속된 식별자
            broadcaster.add(ws)

        async def on_receive(self, ws: WebSocket, data: dict) -> None:
            await dispatcher.dispatch(ws, data)

        async def on_disconnect(self, ws: WebSocket, close_code: int) -> None:
            broadcaster.discard(ws)
            if on_cleanup is not None:
                await on_cleanup(ws)

    return Gateway
