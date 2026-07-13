"""메시지 타입 → 핸들러 디스패처 (도메인 무관, 재사용).

단일 WebSocket에 여러 종류 메시지를 태울 때 if/elif 대신 등록으로 라우팅.

    bus = Dispatcher()

    @bus.on("chat")
    async def _(ws, data): ...

    await bus.dispatch(ws, data)   # data["type"] 보고 자동 라우팅
"""
from __future__ import annotations

from typing import Awaitable, Callable

Handler = Callable[[object, dict], Awaitable[None]]


class Dispatcher:
    def __init__(self) -> None:
        self._handlers: dict[str, Handler] = {}

    def on(self, mtype: str) -> Callable[[Handler], Handler]:
        def deco(fn: Handler) -> Handler:
            self._handlers[mtype] = fn
            return fn
        return deco

    async def dispatch(self, ws: object, data: dict) -> None:
        fn = self._handlers.get(data.get("type"))
        if fn is not None:
            await fn(ws, data)
        # 미등록 타입은 무시 (원하면 여기서 로깅)
