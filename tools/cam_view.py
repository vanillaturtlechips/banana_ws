#!/usr/bin/env python3
"""RealSense D455 라이브 뷰어 — 촬영 해상도/거리/대상 크기 눈으로 검증용.

- 컬러 1280x800 + 뎁스(컬러에 정렬) 나란히 표시
- 화면 중앙 십자 + 그 지점 실제 거리(m)
- "현재 중앙 거리에서 16cm 바나나가 차지할 px" 기준 박스(초록) 오버레이
  → 70~80cm 겨눴을 때 이 박스 크기 = 재수집 시 바나나가 잡히는 크기
키:  s=스냅샷 저장(~/banana_ws/cam_snap_*.png)   q/ESC=종료
"""
import pyrealsense2 as rs
import numpy as np
import cv2, time, os

BANANA_M = 0.16  # 바나나 기준 길이(m)

pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 1280, 800, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
prof = pipe.start(cfg)
align = rs.align(rs.stream.color)
intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
fx = intr.fx
print(f"color {intr.width}x{intr.height} fx={fx:.1f} | s=저장 q=종료")

try:
    while True:
        frames = align.process(pipe.wait_for_frames())
        c = frames.get_color_frame(); d = frames.get_depth_frame()
        if not c or not d:
            continue
        color = np.asanyarray(c.get_data())
        depth = np.asanyarray(d.get_data())
        H, W = color.shape[:2]
        cxp, cyp = W // 2, H // 2

        dist = d.get_distance(cxp, cyp)  # 중앙 거리(m)
        depth_vis = cv2.applyColorMap(
            cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)
        depth_vis = cv2.resize(depth_vis, (W, H))

        # 중앙 십자
        cv2.drawMarker(color, (cxp, cyp), (0, 0, 255), cv2.MARKER_CROSS, 40, 2)
        # 기준 박스: 현재 중앙 거리에서 16cm가 몇 px인가
        if dist > 0:
            ban_px = int(fx * BANANA_M / dist)
            x1, y1 = cxp - ban_px // 2, cyp - ban_px // 2
            cv2.rectangle(color, (x1, y1), (x1 + ban_px, y1 + ban_px), (0, 255, 0), 2)
            txt = f"dist={dist*100:.0f}cm  banana(16cm)~{ban_px}px ({ban_px/W*100:.1f}% of frame)"
        else:
            txt = "dist=--- (뎁스 없음: 너무 가깝/멀거나 반사)"
        cv2.putText(color, txt, (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0), 2, cv2.LINE_AA)

        view = np.hstack([color, depth_vis])
        view = cv2.resize(view, (view.shape[1] // 2, view.shape[0] // 2))
        cv2.imshow("D455  [color | depth]  (s=save q=quit)", view)
        k = cv2.waitKey(1) & 0xFF
        if k in (ord('q'), 27):
            break
        if k == ord('s'):
            fn = os.path.expanduser(f"~/banana_ws/cam_snap_{int(time.time())}.png")
            cv2.imwrite(fn, color)
            print("saved", fn)
finally:
    pipe.stop()
    cv2.destroyAllWindows()
