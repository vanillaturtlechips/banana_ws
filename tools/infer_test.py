#!/usr/bin/env python3
"""주사위 실시간 추론 테스트 — best.pt + 시간적 안정화(temporal voting).

매 프레임 추론하되, 같은 클래스가 STREAK 프레임 연속으로 top일 때만 '확정'.
  - 확정 전: 노란 "판단중" (AE/WB 흔들리는 0~3초 구간을 넘김)
  - 확정 후: 초록 "CONFIRMED"
굴릴 때마다:
    SPACE = 현재 '확정' 결과 기록/저장  (확정 안됐으면 unstable로 표시)
    q/ESC/Ctrl+C = 종료
저장: /home/user/Pictures/roll_test_voted/roll_NN.jpg (user 소유)
"""
import pyrealsense2 as rs
import numpy as np
import cv2, os
from collections import deque, Counter
from ultralytics import YOLO

MODEL = "/root/banana_ws/src/banana_perception/models/best.pt"  # 배포된 최신 모델
OUT = "/home/user/Pictures/roll_test_voted"
CONF = 0.5
STREAK = 12        # 같은 클래스 연속 프레임 수 (~0.4s @30fps) → 확정
WINDOW = 15        # 다수결 참고 창
os.makedirs(OUT, exist_ok=True); os.chown(OUT, 1000, 1000)

model = YOLO(MODEL); names = model.names
print("classes:", names, "| SPACE=기록 q=종료")

pipe = rs.pipeline(); cfg = rs.config()
cfg.enable_stream(rs.stream.color, 1280, 800, rs.format.bgr8, 30)
pipe.start(cfg)

def top_pred(frame):
    r = model(frame, conf=CONF, verbose=False)[0]
    best = None
    for b in r.boxes:
        conf = float(b.conf[0]); cls = names[int(b.cls[0])]
        xy = [int(v) for v in b.xyxy[0]]
        if best is None or conf > best[1]:
            best = (cls, conf, xy)
    return best

hist = deque(maxlen=WINDOW)   # 최근 top 클래스들
saved = 0
try:
    while True:
        frames = pipe.wait_for_frames()
        c = frames.get_color_frame()
        if not c:
            continue
        frame = np.asanyarray(c.get_data())
        best = top_pred(frame)
        cur = best[0] if best else None
        hist.append(cur)

        # 확정 판정: 마지막 STREAK개가 모두 같은 (None 아닌) 클래스
        recent = list(hist)[-STREAK:]
        confirmed = recent[0] if (len(recent) == STREAK and cur is not None
                                  and all(x == cur for x in recent)) else None
        # 안정화 진행도
        streak_n = 0
        for x in reversed(list(hist)):
            if x == cur and cur is not None: streak_n += 1
            else: break

        vis = frame.copy()
        if best:
            x1, y1, x2, y2 = best[2]
            col = (0, 200, 0) if confirmed else (0, 210, 210)   # 초록/노랑
            cv2.rectangle(vis, (x1, y1), (x2, y2), col, 3)
            cv2.putText(vis, f"{best[0]} {best[1]:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, col, 2, cv2.LINE_AA)
        if confirmed:
            hud, hcol = f"CONFIRMED: {confirmed}", (0, 220, 0)
        else:
            hud, hcol = f"...stabilizing {min(streak_n,STREAK)}/{STREAK}", (0, 210, 210)
        cv2.putText(vis, f"[SPACE=save q=quit] saved={saved}  {hud}",
                    (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.8, hcol, 2, cv2.LINE_AA)

        cv2.imshow("infer test (voted)", cv2.resize(vis, (vis.shape[1]//2, vis.shape[0]//2)))
        k = cv2.waitKey(1) & 0xFF
        if k in (ord('q'), 27):
            break
        if k == 32:  # SPACE
            label = confirmed or f"UNSTABLE({cur})"
            vote = Counter(x for x in hist if x).most_common(1)
            fn = os.path.join(OUT, f"roll_{saved:02d}.jpg")
            cv2.imwrite(fn, vis); os.chown(fn, 1000, 1000)
            tag = "OK " if confirmed else "!! "
            print(f"{tag}[{saved:02d}] {label:12} "
                  f"(vote {vote[0] if vote else '-'}) -> {fn}")
            saved += 1
except KeyboardInterrupt:
    pass
finally:
    pipe.stop(); cv2.destroyAllWindows()
    print(f"\n총 {saved}장 기록: {OUT}")
