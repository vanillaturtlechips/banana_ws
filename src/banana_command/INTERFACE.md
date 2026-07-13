## Seam 1 — 명령 OUT (브리지 → YOLO)

| 항목 | 값 |
|---|---|
| 인터페이스 | `banana_command/msg/SortCommand` (topic) |
| 토픽명 | `/banana/command` |
| 방향 | 브리지 publish → YOLO subscribe |
| QoS | reliable, keep_last 10 |

```
string    action          # sort | pick_only | stop | rescan
string[]  target_stages   # unripe|ripe|overripe|rotten (빈 값=전체)
string    params_json     # 자유 파라미터 JSON
string    source_utterance
string    session_id
```

종수님 확인 필요: 이 필드로 충분한가? (예: 개수 제한, 우선순위 등 추가 필요?)

---

## Seam 2 — 상태 IN (파이프라인 → 브리지) aggregator 소유?

브리지는 통합 상태 토픽 1개만 구독하고 싶다(결합도 최소를 최소화했어요)
누가 aggregator를 소유할지 결정 필요해요

| 항목 | 값(제안) |
|---|---|
| 토픽명 | `/banana/system_state` |
| 방향 | aggregator publish → 브리지 subscribe |
| QoS | reliable, keep_last 5 |
| 주기 | 2~5 Hz (throttle) |

필요 필드 (프론트 LiveState 매핑):
| 프론트 필드 | 의미 | 출처 |
|---|---|---|
| `robot` | ready/picking/placing/idle/error | MoveIt2 실행 상태 |
| `robotMessage` | 사람이 읽을 상태 문구 | 파이프라인 |
| `detection` | stage, confidence, box{x,y,w,h}(%) | YOLO |
| `logs[]` | 분류 이력 | 파이프라인 |
| `exceptions[]` | 저신뢰/실패 케이스 | 파이프라인 |

미정: aggregator 없으면 브리지가 N개 토픽 직접 구독으로

---

## Seam 3 (선택) — 영상 IN (카메라 → 브리지)

WebRTC 송출용. 원본 대신 다운스케일/압축 권장.

| 항목 | 값 |
|---|---|
| 토픽명 | `/camera/color/image_raw` (또는 `.../compressed`) |
| 타입 | `sensor_msgs/Image` |
| QoS | **best_effort**, keep_last 1 (sensor_data) |
| encoding | rgb8 / bgr8 (확인 필요 — 색 반전 방지) |

---

## 결정해야 할 것 (회의 안건)
1. `SortCommand` 필드 승인 / 추가
2. aggregator 소유자 — 누구? (없으면 다른 방향)
3. `system_state` 최종 필드명 + 타입 패키지명
4. 카메라 토픽 encoding + 압축 여부
5. 각 토픽 QoS 프로파일 최종 확정
