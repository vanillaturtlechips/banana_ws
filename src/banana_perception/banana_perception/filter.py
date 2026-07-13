"""SortCommand 필터 + 대상 선택 (ROS 무관, 순수 Python)."""
from __future__ import annotations

from typing import List, Optional, Sequence

from .detector import Det


def select_target(
    dets: List[Det],
    target_stages: Sequence[str],
    min_conf: float = 0.0,
) -> Optional[Det]:
    """현재 프레임 감지들 중, 명령에 맞는 것 하나를 고른다.

    - target_stages 비었으면 = 전체 대상
    - confidence 최고를 선택 (동률이면 첫 번째)
    - 후보 없으면 None
    """
    cands = [
        d for d in dets
        if (not target_stages or d.stage in target_stages) and d.confidence >= min_conf
    ]
    if not cands:
        return None
    return max(cands, key=lambda d: d.confidence)
