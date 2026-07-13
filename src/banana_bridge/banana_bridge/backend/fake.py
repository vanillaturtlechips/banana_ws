"""페이크 백엔드 — rclpy/하드웨어 없이 브리지 전체를 돌린다.

프론트 banana-mock.ts의 파이프라인(detection→pick→place→log)을 포팅해서
가짜 LiveState를 뿜고, 합성 영상 프레임을 FrameSource에 넣는다.
실물 준비되면 app.py에서 BridgeRos로 교체만 하면 됨 (인터페이스 동일).
"""
from __future__ import annotations

import asyncio
import math
import random
import time
from typing import AsyncIterator, Optional

import numpy as np

from ..domain.schema import SortCommandModel
from ..transport.webrtc import FrameSource
# Backend(Protocol) 을 구조적으로 만족 — app.py에서 계약 확인.

STAGES = ["unripe", "ripe", "overripe", "rotten"]
MESSAGES = {
    "unripe": "초록 바나나 발견! 후숙 보관함으로 옮기는 중이에요 🌱",
    "ripe": "잘 익은 바나나 발견! 판매함으로 이동합니다 🍌",
    "overripe": "갈색 바나나예요. 가공용 보관함으로 옮길게요 🥣",
    "rotten": "아쉽지만 썩었어요. 쓰레기통으로 이동합니다 🗑️",
}


class FakeBridge:
    def __init__(self, frame_source: FrameSource) -> None:
        self._fs = frame_source
        self._logs: list[dict] = []
        self._q: "asyncio.Queue[dict]" = asyncio.Queue()
        self._tasks: list[asyncio.Task] = []
        self._override_stage: Optional[str] = None  # 명령이 오면 그 단계만 생성

    # ---- 수명주기 (app 인터페이스) --------------------------------
    def start(self) -> None:
        self._tasks = [
            asyncio.create_task(self._pipeline()),
            asyncio.create_task(self._video()),
        ]

    def stop(self) -> None:
        for t in self._tasks:
            t.cancel()

    # ---- OUT: 명령 수신 -------------------------------------------
    def publish_command(self, model: SortCommandModel) -> None:
        # 실물이면 여기서 ROS publish. 페이크는 파이프라인에 반영만.
        if model.action.value == "stop":
            self._override_stage = "__stopped__"
        elif model.action.value == "sort" and model.target_stages:
            self._override_stage = model.target_stages[0].value
        else:
            self._override_stage = None
        print(f"[fake] command: {model.action.value} {[s.value for s in model.target_stages]}")

    # ---- IN: 상태 스트림 ------------------------------------------
    async def state_stream(self) -> AsyncIterator[dict]:
        while True:
            yield await self._q.get()

    # ---- 내부: 가짜 파이프라인 ------------------------------------
    async def _pipeline(self) -> None:
        await self._emit("ready", "바나나를 기다리는 중...", None)
        while True:
            if self._override_stage == "__stopped__":
                await self._emit("idle", "정지됨", None)
                await asyncio.sleep(1.5)
                continue

            stage = self._override_stage or random.choice(STAGES)
            conf = 0.82 + random.random() * 0.17
            det = {
                "stage": stage,
                "confidence": conf,
                "box": {
                    "x": 20 + random.random() * 20,
                    "y": 18 + random.random() * 20,
                    "w": 40 + random.random() * 15,
                    "h": 45 + random.random() * 15,
                },
            }
            await self._emit("picking", MESSAGES[stage], det)
            await asyncio.sleep(1.2)
            await self._emit("placing", MESSAGES[stage], det)
            await asyncio.sleep(0.9)

            self._logs.insert(0, {
                "id": f"{int(time.time()*1000)}-{random.randint(0,9999)}",
                "stage": stage,
                "confidence": conf,
                "at": time.time(),
                "status": "review" if conf < 0.7 else "ok",
            })
            self._logs = self._logs[:20]
            await self._emit("ready", "다음 바나나 대기 중...", None)
            await asyncio.sleep(2.5)

    async def _emit(self, robot: str, message: str, detection: Optional[dict]) -> None:
        await self._q.put({
            "robot": robot,
            "robotMessage": message,
            "detection": detection,
            "logs": self._logs,
            "exceptions": [],
        })

    # ---- 내부: 합성 영상 (WebRTC 테스트용) -----------------------
    async def _video(self) -> None:
        """움직이는 그라디언트 — 카메라 없이 WebRTC 파이프 검증."""
        t0 = time.time()
        while True:
            t = time.time() - t0
            h, w = 480, 640
            xs = np.linspace(0, 1, w)
            shift = (math.sin(t) + 1) / 2
            r = np.clip((xs + shift) % 1.0, 0, 1)
            row = np.stack([r, np.full(w, 0.85), np.full(w, 0.2)], axis=1)
            frame = np.tile(row, (h, 1, 1))
            self._fs.put((frame * 255).astype(np.uint8))
            await asyncio.sleep(1 / 15)
