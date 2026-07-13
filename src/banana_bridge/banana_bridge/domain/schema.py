"""LLM 출력 JSON 스키마 = SortCommand.msg 과 1:1. pydantic으로 강제 검증.

LLM structured output(JSON mode)의 스키마로도 그대로 사용.
"""
from __future__ import annotations

from enum import Enum
from typing import Optional

from pydantic import BaseModel, Field


class Stage(str, Enum):
    unripe = "unripe"
    ripe = "ripe"
    overripe = "overripe"
    rotten = "rotten"


class Action(str, Enum):
    sort = "sort"
    pick_only = "pick_only"
    stop = "stop"
    rescan = "rescan"


class SortCommandModel(BaseModel):
    action: Action
    target_stages: list[Stage] = Field(default_factory=list)  # 빈 값 = 전체
    params: dict = Field(default_factory=dict)

    # LLM이 스키마 밖 값을 뱉으면 여기서 ValidationError → 폴백/재질의 트리거
