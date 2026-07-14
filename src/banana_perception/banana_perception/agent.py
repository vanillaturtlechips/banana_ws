"""공간 셀렉터 + 실현가능성 게이트 (ROS 무관, 순수 로직).

파이프라인: LLM selector({by, stage?}) + 현재 감지들(Detection 리스트)
  1) select_object : 조건에 맞는 감지 1개 선택
  2) check_feasibility : 라우팅 정책으로 집기 가능/차단 + 목적지 통

감지 객체는 .stage/.confidence/.center_x/.center_y 를 가지면 됨(Detection msg or Det).
nearest/farthest 는 grasp_pose(base_link) 있으면 base거리, 없으면 center_y proxy.
"""
from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

# 확정 라우팅 정책 (memory: banana-routing-policy)
#   unripe = 집지 않음, ripe/overripe → 1번 보관함, rotten → 2번(쓰레기통)
ROUTING = {
    "unripe": None,       # pick 대상 아님
    "ripe": "bin1",
    "overripe": "bin1",
    "rotten": "bin2",
}
PICKABLE = {s for s, b in ROUTING.items() if b is not None}

BY_OPTIONS = ("leftmost", "rightmost", "nearest", "farthest", "stage")


def _base_dist(d) -> Optional[float]:
    """로봇 base 기준 XY 거리(m). grasp_pose(base_link) 있을 때만. 없으면 None."""
    if getattr(d, "has_pose", False):
        p = d.grasp_pose.pose.position
        return math.hypot(p.x, p.y)
    return None


def select_object(dets: Sequence, by: str, stage: Optional[str] = None):
    """감지들 중 by 기준으로 1개 선택. 후보 없으면 None.

    leftmost/rightmost = center_x, stage = 해당 stage 최고신뢰,
    nearest/farthest = base거리(있으면)/center_y proxy(아래=가까움).
    """
    cands = [d for d in dets if (stage is None or d.stage == stage)]
    if not cands:
        return None
    if by == "stage":
        return max(cands, key=lambda d: d.confidence)
    if by == "leftmost":
        return min(cands, key=lambda d: d.center_x)
    if by == "rightmost":
        return max(cands, key=lambda d: d.center_x)
    if by in ("nearest", "farthest"):
        if all(_base_dist(d) is not None for d in cands):
            key = _base_dist                       # base 거리(m)
        else:
            key = lambda d: -d.center_y            # cy 클수록 화면 아래=가까움
        return (min if by == "nearest" else max)(cands, key=key)
    raise ValueError(f"unknown selector 'by': {by}")


def check_feasibility(det) -> Tuple[bool, str, Optional[str]]:
    """선택된 감지를 집을 수 있나 → (ok, 사유, 목적지통).

    None → 차단, unripe → 차단(안 집는 대상), 그 외 → 통과+bin.
    """
    if det is None:
        return False, "집을 대상이 없어요", None
    dest = ROUTING.get(det.stage)
    if dest is None:
        return False, f"'{det.stage}'는 집지 않는 대상이에요", None
    return True, "", dest


def plan_pick(dets: Sequence, by: str, stage: Optional[str] = None):
    """셀렉터 → 게이트 통합. → (target|None, ok, reason, bin)."""
    target = select_object(dets, by, stage)
    ok, reason, dest = check_feasibility(target)
    return target, ok, reason, dest
