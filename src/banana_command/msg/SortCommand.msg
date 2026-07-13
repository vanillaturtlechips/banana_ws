# LLM이 자연어에서 뽑아낸 구조화 명령입니다 브리지 → YOLO(perception)로 발행했어요

std_msgs/Header header

# 무엇을 할지: "sort" | "pick_only" | "stop" | "rescan"
string action

# 대상 익음 단계 (STAGES id): "unripe" | "ripe" | "overripe" | "rotten"
# 비어 있으면 "전체" 의미
string[] target_stages

# 자유 파라미터 (예: {"max_items": 5}) — JSON 문자열
string params_json

# 추적용: LLM 원문 + 세션 id
string source_utterance
string session_id
