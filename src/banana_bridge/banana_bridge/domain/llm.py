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
    "You parse Korean commands for a banana-sorting robot (camera + arm). "
    "target_stages holds ONLY ripeness values: unripe/ripe/overripe/rotten. "
    "Spatial words (왼쪽/오른쪽/가까운/먼) go in params.by, NEVER in target_stages. "
    "ONLY these four actions produce JSON: pick_only(집기/옮기기), sort(분류), stop(멈춤), rescan(재스캔). "
    "For a control command reply with ONE JSON line. "
    "For ANY question, status/count query, or chit-chat (예: '어떤 상태야?', '몇 개야?', '안녕') "
    "reply with a brief Korean sentence and NEVER output JSON.\n"
    "Examples:\n"
    '가장 왼쪽 바나나 집어줘 => {"action":"pick_only","target_stages":[],"params":{"by":"leftmost"}}\n'
    '오른쪽 거 옮겨줘 => {"action":"pick_only","target_stages":[],"params":{"by":"rightmost"}}\n'
    '제일 가까운 거 잡아 => {"action":"pick_only","target_stages":[],"params":{"by":"nearest"}}\n'
    '익은 것만 골라줘 => {"action":"sort","target_stages":["ripe"],"params":{}}\n'
    '썩은 거 분류해 => {"action":"sort","target_stages":["rotten"],"params":{}}\n'
    '멈춰 => {"action":"stop","target_stages":[],"params":{}}\n'
    "If a [현재 감지] block is given (왼쪽→오른쪽 순서 리스트), use it to answer "
    "status questions (뭐야 / 어떤 상태야 / 왼쪽·오른쪽 뭐야 / 몇 개야) in Korean.\n"
    '오른쪽 거는 어떤 상태야? => (감지 참고) 오른쪽 건 안 익음(unripe)이에요 🍌\n'
    '안녕 => 안녕하세요! 무엇을 도와드릴까요? 🍌'
)


async def parse_utterance(text: str, scene: str = "") -> Tuple[Optional[SortCommandModel], str]:
    raw = await _dispatch(text, scene)

    # JSON이면 명령, 아니면 자연어 답변
    model = _try_parse(raw)
    if model is not None:
        # 공간 명령인데 LLM이 by를 빠뜨리면 키워드로 보완 (7B 신뢰성 보정)
        if model.action.value == "pick_only" and not model.params.get("by"):
            by = _infer_by(text)
            if by:
                model.params["by"] = by
        return model, _confirm_text(model)
    # JSON 시도였으나 검증 실패(잘못된 action 등) → 원문 노출 대신 안내
    if raw.strip().startswith("{"):
        return None, ('그 명령은 아직 이해하지 못했어요. '
                      '"왼쪽 거 집어줘", "익은 것만 분류해줘"처럼 말해보세요 🍌')
    return None, raw


def _infer_by(text: str) -> Optional[str]:
    """텍스트에서 공간 selector 키워드 추출 (LLM 보완/폴백)."""
    t = text.lower()
    return ("leftmost" if ("왼쪽" in t or "좌측" in t) else
            "rightmost" if ("오른쪽" in t or "우측" in t) else
            "nearest" if "가까" in t else
            "farthest" if ("먼 " in t or "멀리" in t or "먼거" in t) else None)


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

async def _dispatch(text: str, scene: str = "") -> str:
    if PROVIDER == "api":
        try:
            return await _call_api(text, scene)
        except Exception:  # 전용망이라 드물지만 폴백
            return await _call_local(text, scene)
    if PROVIDER == "local":
        return await _call_local(text, scene)
    return _call_fake(text)


async def _call_api(text: str, scene: str = "") -> str:
    content = text if not scene else f"[현재 감지]\n{scene}\n\n{text}"
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
                "messages": [{"role": "user", "content": content}],
            },
        )
        r.raise_for_status()
        return r.json()["content"][0]["text"]


async def _call_local(text: str, scene: str = "") -> str:
    prompt = text if not scene else f"[현재 감지]\n{scene}\n\n{text}"
    async with httpx.AsyncClient(timeout=30) as client:
        r = await client.post(
            f"{LOCAL_ENDPOINT}/api/generate",
            json={
                "model": os.getenv("BANANA_LLM_LOCAL_MODEL", "llama3.1"),
                "system": SYSTEM_PROMPT,
                "prompt": prompt,
                "stream": False,
            },
        )
        r.raise_for_status()
        return r.json().get("response", "")


def _call_fake(text: str) -> str:
    """API 키 없이 개발용 — 키워드 규칙 기반 파서."""
    t = text.lower()
    # 공간 선택: "가장 왼쪽 잡아줘" 등 → params.by
    by = ("leftmost" if ("왼쪽" in t or "좌측" in t) else
          "rightmost" if ("오른쪽" in t or "우측" in t) else
          "nearest" if "가까" in t else
          "farthest" if ("먼 " in t or "멀리" in t or "먼거" in t) else None)
    if by and any(k in t for k in ("잡", "집", "옮")):
        return json.dumps({"action": "pick_only", "target_stages": [],
                           "params": {"by": by}})
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
