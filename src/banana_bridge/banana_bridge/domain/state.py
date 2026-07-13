"""system_state(ROS) → 프론트 LiveState(JSON) 순수 변환기.

⚠️ 로직 금지. 게이트웨이는 매핑만 한다. 판단이 필요하면 그건 aggregator 몫.
프론트 스키마: src/lib/banana-mock.ts 의 LiveState 참고.
"""
from __future__ import annotations

from typing import Any


def to_live_state(system_state: Any) -> dict:
    """TODO: aggregator SystemState 필드 확정되면 실제 매핑.

    반환 형태(프론트 계약):
      {
        "robot": "ready|picking|placing|idle|error",
        "robotMessage": str,
        "detection": {"stage","confidence","box":{x,y,w,h}} | None,
        "logs": [...], "exceptions": [...]
      }
    """
    # 아래는 뼈대 예시 — 실제 필드명으로 교체
    return {
        "robot": getattr(system_state, "robot_status", "idle"),
        "robotMessage": getattr(system_state, "message", ""),
        "detection": _map_detection(system_state),
        "logs": [],        # TODO
        "exceptions": [],  # TODO
    }


def _map_detection(system_state: Any) -> dict | None:
    det = getattr(system_state, "detection", None)
    if det is None:
        return None
    return {
        "stage": det.stage,
        "confidence": det.confidence,
        "box": {"x": det.box.x, "y": det.box.y, "w": det.box.w, "h": det.box.h},
    }
