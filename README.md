# 🍌 BananaSort — Bridge & Backend

바나나 익음 4단계(안익음·적당·너무익음·썩음) 분류 로봇의 **백엔드 모노레포**입니다.
자연어 명령을 받아 로봇 파이프라인으로 넘기고, 실시간 상태·카메라 영상을 웹으로 중계합니다.

---

## 전체 파이프라인

```
자연어 → LLM → JSON → YOLO → Hand-eye 좌표변환 → MoveIt2 → Pick & Place
─────── 이 repo(bridge) ───────┘   └──────── 다른 팀 담당 ───────────┘
```

`bridge`는 **웹 게이트웨이**입니다. 하는 일은 2가지:
- **명령 OUT**: 채팅 자연어 → LLM으로 JSON 명령(`SortCommand`) 변환 → 파이프라인으로 발행
- **상태 IN**: 로봇/감지 상태를 모아 웹 프론트로 실시간 push (+ 카메라 영상 WebRTC)

```
[웹 프론트] ⟷ WebSocket ⟷ [bridge] ⟶ SortCommand ⟶ [YOLO → ... → 로봇팔]
   (Lovable)                   ⟵ system_state ⟵ (상태 집계)
```

---

## 레포 구조 (모노레포)

```
banana_ws/
└── src/
    ├── banana_command/      # 📜 인터페이스 계약 (SortCommand.msg + INTERFACE.md)
    │                        #    → 모든 팀이 공유. 변경 시 합의 필요!
    ├── banana_bridge/       # 🌉 웹 게이트웨이 (이 repo의 핵심)
    ├── banana_perception/   # 👁  YOLO 감지 (담당: ○○○)   ← 추가 예정
    └── banana_moveit/       # 🦾 MoveIt2 + 두산팔 (담당: ○○○) ← 추가 예정
```

> 프론트엔드는 **별도 repo**(`fruity-arm-showcase`, Lovable)에 있습니다.
> 코드가 아니라 **WebSocket JSON 계약**으로만 연결됩니다.

### bridge 내부 구조
| 폴더 | 역할 |
|---|---|
| `transport/` | ⭐ 재사용 배관 (도메인 무관): WebRTC 영상 + WS 게이트웨이/디스패처/방송 |
| `backend/` | 스왑 가능한 백엔드: `base`(계약) · `fake`(ROS 없이) · `ros`(실물 rclpy) |
| `domain/` | 바나나 전용: `schema`(명령) · `llm`(자연어→JSON) · `state`(상태 매핑) |
| `app.py` | 조립 루트 (엮기만) |

자세한 건 [`src/banana_bridge/banana_bridge/README.md`](src/banana_bridge/banana_bridge/README.md).

---

## 빠른 시작 (ROS·하드웨어 없이 실행) 🚀

**페이크 모드**로 로봇/카메라/ROS 없이 브리지 전체가 돌아갑니다.

```bash
cd ~/banana_ws/src/banana_bridge
pip install -r requirements.txt
BANANA_FAKE=1 uvicorn banana_bridge.app:app --host 0.0.0.0 --port 8000
```

- 가짜 상태(분류 사이클)와 합성 영상이 흘러나옵니다.
- 프론트를 붙이면 채팅·실시간 화면이 그대로 동작합니다.

### 프론트 연결
```bash
cd ~/fruity-arm-showcase   # 별도 repo
bun install && bun dev
```
브리지 주소는 프론트 환경변수 `VITE_BRIDGE_WS`(기본 `ws://<host>:8000/ws`).

---

## 실물 모드 (ROS2 + 로봇)

```bash
cd ~/banana_ws
pip install -r src/banana_bridge/requirements.txt          # pip 의존
rosdep install --from-paths src --ignore-src -r -y          # ROS 의존
colcon build && source install/setup.bash
BANANA_FAKE=0 uvicorn banana_bridge.app:app --port 8000
```

> ⚠️ 실물 모드는 `domain/state.py`, `backend/ros.py`의 **TODO**를 먼저 채워야 합니다.
> (aggregator의 `system_state` 필드 확정 후 — [INTERFACE.md](src/banana_command/INTERFACE.md) 참고)

---

## 환경변수

| 변수 | 기본값 | 설명 |
|---|---|---|
| `BANANA_FAKE` | `1` | `1`=페이크(ROS 없이) / `0`=실물 rclpy |
| `BANANA_LLM_PROVIDER` | `fake` | `fake`(키 불필요) / `api`(클라우드) / `local`(Ollama) |
| `BANANA_LLM_API_KEY` | — | `api` 모드용 키 (⚠️ 코드/yaml에 넣지 말 것) |
| `BANANA_LLM_MODEL` | `claude-opus-4-8` | API 모델명 |
| `BANANA_LLM_LOCAL` | `http://localhost:11434` | Ollama 엔드포인트 |

---

## 팀 협업 규칙

1. **`banana_command`(인터페이스)는 공유 계약** — 바꾸기 전 반드시 관련 팀과 합의.
   변경 절차·미정 안건은 [INTERFACE.md](src/banana_command/INTERFACE.md)에 정리됨.
2. **브랜치 → PR** 로 작업. `main`은 항상 빌드되는 상태 유지.
3. **비밀(API 키 등)은 커밋 금지** — `.env`로 관리 (`.gitignore`에 포함됨).
4. 프론트(`fruity-arm-showcase`)는 **Lovable 동기화 repo** — force-push/히스토리 재작성 금지.

---

## 담당

| 영역 | 담당 |
|---|---|
| bridge (이 repo) · LLM · 프론트 연결 | (본인) |
| YOLO 감지 | ○○○ |
| Hand-eye 좌표변환 · MoveIt2 · 두산팔 | ○○○ |
| 상태 집계(aggregator) | 미정 — [INTERFACE.md](src/banana_command/INTERFACE.md) 참고 |

---

## 핵심 설계 결정

- **페이크/실물 스왑**: `BANANA_FAKE`로 백엔드 교체 → 하드웨어 없이 개발·데모 가능
- **느슨한 결합**: 프론트↔브리지는 WebSocket JSON 계약으로만 연결 (코드 의존 X)
- **영상/상태 분리**: 카메라 영상은 WebRTC, 상태·bbox는 WebSocket JSON 오버레이
- **재사용 배관**: `transport/`는 도메인을 몰라서 다른 로봇 프로젝트에 그대로 복사 가능
