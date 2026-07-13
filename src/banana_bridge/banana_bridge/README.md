# banana_bridge 구조

| 항목 | 역할 |
|---|---|
| `app.py` | 조립 루트. 배관+백엔드+도메인을 엮고 핸들러 등록·수명주기만. 로직 없음 |
| `transport/` | ⭐ 재사용 배관 (도메인 무관). WebRTC 송출 + WS 게이트웨이/디스패처/방송 |
| `backend/` | 스왑 가능한 백엔드. `base.py`(계약) · `fake.py`(ROS 없이) · `ros.py`(실물 rclpy) |
| `domain/` | 🍌 바나나 전용. `schema`(명령) · `llm`(자연어→JSON) · `state`(LiveState 매핑) |
| `__init__.py` | 패키지 표식 (빈 파일). 각 폴더를 파이썬 패키지로 인식시킴 |

의존 방향: `app → transport / backend / domain`, `backend → domain + transport`
(`transport`는 아무것도 import 안 함 = 그대로 복붙 재사용 가능)

실행: `BANANA_FAKE=1 uvicorn banana_bridge.app:app --port 8000`
