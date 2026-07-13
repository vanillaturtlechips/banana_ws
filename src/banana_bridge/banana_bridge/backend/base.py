"""백엔드 계약 — fake ↔ real 스왑 지점(seam).

FakeBridge, BridgeRos 둘 다 이 Protocol을 구조적으로 만족해야 한다.
app.py는 어느 구현이든 이 인터페이스로만 다룬다.
"""
from __future__ import annotations

from typing import AsyncIterator, Protocol, runtime_checkable

from ..domain.schema import SortCommandModel


@runtime_checkable
class Backend(Protocol):
    def start(self) -> None:
        """구독/스레드/파이프라인 시작."""
        ...

    def stop(self) -> None:
        """정리."""
        ...

    def publish_command(self, model: SortCommandModel) -> None:
        """LLM이 파싱한 명령을 파이프라인(→YOLO)으로 전달."""
        ...

    def state_stream(self) -> AsyncIterator[dict]:
        """프론트로 보낼 LiveState dict를 비동기로 흘린다."""
        ...
