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


# --- aggregator 우회: banana_command/Detection 메시지 → LiveState 직접 매핑 ---
# aggregator가 생기면 이 경로를 to_live_state(SystemState)로 교체.
def detection_msg_to_live_state(det: Any) -> dict:
    """Detection.msg 하나 → 프론트 LiveState(JSON).

    로봇/aggregator 없음 → robot 필드는 스텁. box는 픽셀 → % (프론트 계약).
    """
    return {
        "robot": "idle",
        "robotMessage": f"{det.stage} 감지 ({det.confidence:.2f})",
        "detection": _map_detection_msg(det),
        "logs": [],
        "exceptions": [],
    }


def _map_detection_msg(det: Any) -> dict:
    iw = det.image_width or 1
    ih = det.image_height or 1
    out = {
        "stage": det.stage,
        "confidence": round(float(det.confidence), 3),
        "box": {  # 픽셀 → % (좌상단 x,y + w,h)
            "x": round(det.x / iw * 100, 2),
            "y": round(det.y / ih * 100, 2),
            "w": round(det.w / iw * 100, 2),
            "h": round(det.h / ih * 100, 2),
        },
    }
    # 3D 스키마 필드가 있으면 함께 실어보냄 (없으면 생략 — 2D 노드 호환)
    if hasattr(det, "angle_deg"):
        out["angle_deg"] = round(float(det.angle_deg), 1)
    if getattr(det, "has_pose", False):
        p = det.grasp_pose.pose.position
        out["graspPose"] = {"x": round(p.x, 4), "y": round(p.y, 4), "z": round(p.z, 4)}
    return out
