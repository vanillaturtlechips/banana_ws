"""자연어 → 구조화 명령 (ROS 무관, 순수 Python).

반환: (SortCommandModel | None, 사용자에게 보여줄 답변 문자열)
  - 명령이면 model 채워짐
  - 단순 질문/잡담이면 model=None, 답변만

provider = api | local | fake  (env BANANA_LLM_PROVIDER 또는 config)
API 키 없이도 fake 모드로 전체 파이프라인이 돈다.
"""
from __future__ import annotations

import json
import os
from typing import Optional, Tuple

import httpx
from pydantic import ValidationError

from .schema import SortCommandModel

PROVIDER = os.getenv("BANANA_LLM_PROVIDER", "fake")
API_KEY = os.getenv("BANANA_LLM_API_KEY", "")
API_MODEL = os.getenv("BANANA_LLM_MODEL", "claude-opus-4-8")
LOCAL_ENDPOINT = os.getenv("BANANA_LLM_LOCAL", "http://localhost:11434")

SYSTEM_PROMPT = (
    "You parse commands for a banana-sorting robot. "
    "If the message is a control command, reply with ONE JSON line: "
    '{"action":"sort|pick_only|stop|rescan",'
    '"target_stages":["unripe|ripe|overripe|rotten"],"params":{}} '
    "Otherwise reply briefly in Korean, no JSON. "
    "All natural-language replies must be in Korean."
)


async def parse_utterance(text: str) -> Tuple[Optional[SortCommandModel], str]:
    raw = await _dispatch(text)

    # JSON이면 명령, 아니면 자연어 답변
    model = _try_parse(raw)
    if model is None:
        return None, raw
    return model, _confirm_text(model)


def _try_parse(raw: str) -> Optional[SortCommandModel]:
    raw = raw.strip()
    # 코드펜스/앞뒤 잡텍스트 제거해서 첫 { } 블록만 추출
    start, end = raw.find("{"), raw.rfind("}")
    if start == -1 or end <= start:
        return None
    try:
        return SortCommandModel.model_validate_json(raw[start : end + 1])
    except (ValidationError, ValueError):
        return None


def _confirm_text(model: SortCommandModel) -> str:
    stages = ", ".join(s.value for s in model.target_stages) or "전체"
    return f"'{model.action.value}' 명령 접수 (대상: {stages}) 🍌"


# --- provider 분기 -------------------------------------------------

async def _dispatch(text: str) -> str:
    if PROVIDER == "api":
        try:
            return await _call_api(text)
        except Exception:  # 전용망이라 드물지만 폴백
            return await _call_local(text)
    if PROVIDER == "local":
        return await _call_local(text)
    return _call_fake(text)


async def _call_api(text: str) -> str:
    async with httpx.AsyncClient(timeout=15) as client:
        r = await client.post(
            "https://api.anthropic.com/v1/messages",
            headers={
                "x-api-key": API_KEY,
                "anthropic-version": "2023-06-01",
                "content-type": "application/json",
            },
            json={
                "model": API_MODEL,
                "max_tokens": 256,
                "system": SYSTEM_PROMPT,
                "messages": [{"role": "user", "content": text}],
            },
        )
        r.raise_for_status()
        return r.json()["content"][0]["text"]


async def _call_local(text: str) -> str:
    async with httpx.AsyncClient(timeout=30) as client:
        r = await client.post(
            f"{LOCAL_ENDPOINT}/api/generate",
            json={
                "model": os.getenv("BANANA_LLM_LOCAL_MODEL", "llama3.1"),
                "system": SYSTEM_PROMPT,
                "prompt": text,
                "stream": False,
            },
        )
        r.raise_for_status()
        return r.json().get("response", "")


def _call_fake(text: str) -> str:
    """API 키 없이 개발용 — 키워드 규칙 기반 파서."""
    t = text.lower()
    if any(k in t for k in ("멈춰", "정지", "중지", "stop")):
        return '{"action":"stop","target_stages":[],"params":{}}'
    if any(k in t for k in ("다시", "재스캔", "스캔", "rescan")):
        return '{"action":"rescan","target_stages":[],"params":{}}'
    if "썩" in t or "rotten" in t:
        return '{"action":"sort","target_stages":["rotten"],"params":{}}'
    if "익은" in t or "노랑" in t or "ripe" in t:
        return '{"action":"sort","target_stages":["ripe"],"params":{}}'
    if "초록" in t or "안 익" in t:
        return '{"action":"sort","target_stages":["unripe"],"params":{}}'
    if "골라" in t or "분류" in t or "sort" in t:
        return '{"action":"sort","target_stages":[],"params":{}}'
    return "무엇을 도와드릴까요? '익은 것만 골라줘', '멈춰' 처럼 말해보세요 🍌"
