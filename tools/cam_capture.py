#!/usr/bin/env python3
"""RealSense D455 데이터 수집 도구 — 클래스별 고해상도 촬영.

키를 누르면 현재 프레임을 해당 클래스 폴더에 저장:
    u = unripe   r = ripe   o = overripe   t = rotten
    q/ESC = 종료
저장물(클래스별 폴더, ~/banana_ws/captures/<class>/):
    <class>_<idx>.jpg        컬러 1280x800 (학습용)
    <class>_<idx>_depth.npy  정렬된 뎁스(mm, uint16) — 로봇/거리분석용(옵션)
화면에 클래스별 누적 장수 + 중앙 거리 표시.
"""
import pyrealsense2 as rs
import numpy as np
import cv2, os, glob

CLASSES = {ord('u'): "unripe", ord('r'): "ripe",
           ord('o'): "overripe", ord('t'): "rotten"}
ROOT = os.path.expanduser("~/banana_ws/captures")
SAVE_DEPTH = True

for c in CLASSES.values():
    os.makedirs(os.path.join(ROOT, c), exist_ok=True)

def next_idx(cls):
    n = glob.glob(os.path.join(ROOT, cls, f"{cls}_*.jpg"))
    return len(n)

counts = {c: next_idx(c) for c in CLASSES.values()}

pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 1280, 800, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
prof = pipe.start(cfg)
align = rs.align(rs.stream.color)
print(f"저장 위치: {ROOT}")
print("u=unripe r=ripe o=overripe t=rotten | q=종료")

try:
    while True:
        frames = align.process(pipe.wait_for_frames())
        c = frames.get_color_frame(); d = frames.get_depth_frame()
        if not c or not d:
            continue
        color = np.asanyarray(c.get_data())
        depth = np.asanyarray(d.get_data())
        H, W = color.shape[:2]
        dist = d.get_distance(W // 2, H // 2)

        # 오버레이(원본은 깨끗이 저장하려고 복사본에만 그림)
        disp = color.copy()
        cv2.drawMarker(disp, (W // 2, H // 2), (0, 0, 255), cv2.MARKER_CROSS, 40, 2)
        cv2.putText(disp, f"dist={dist*100:.0f}cm", (12, 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2, cv2.LINE_AA)
        y = 70
        for c_ in ["unripe", "ripe", "overripe", "rotten"]:
            cv2.putText(disp, f"{c_}: {counts[c_]}", (12, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2, cv2.LINE_AA)
            y += 30
        cv2.putText(disp, "u/r/o/t=capture  q=quit", (12, H - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2, cv2.LINE_AA)

        view = cv2.resize(disp, (W // 2, H // 2))
        cv2.imshow("D455 capture", view)
        k = cv2.waitKey(1) & 0xFF
        if k in (ord('q'), 27):
            break
        if k in CLASSES:
            cls = CLASSES[k]
            idx = counts[cls]
            base = os.path.join(ROOT, cls, f"{cls}_{idx:03d}")
            cv2.imwrite(base + ".jpg", color)            # 깨끗한 원본 저장
            if SAVE_DEPTH:
                np.save(base + "_depth.npy", depth)
            counts[cls] = idx + 1
            print(f"saved {cls}_{idx:03d}.jpg  (dist={dist*100:.0f}cm)")
finally:
    pipe.stop()
    cv2.destroyAllWindows()
    print("\n최종 수집:", counts)
