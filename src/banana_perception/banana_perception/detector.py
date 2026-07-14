"""YOLO 추론 래퍼 (ROS 무관, 순수 Python).

  - Det            : 감지 결과 하나 (dataclass)
  - YoloDetector   : ultralytics 모델 로드 → detect(frame)
  - StubDetector   : 모델/GPU 없이 랜덤 결과 (폴백 — 파이프라인 검증용)
  - make_detector  : 모델 파일 있으면 Yolo, 없으면 Stub 자동 선택

ultralytics/torch는 지연 import → 미설치 환경에서도 Stub로 노드가 뜬다.
"""
from __future__ import annotations

import os
import random
from dataclasses import dataclass
from typing import List, Sequence

import numpy as np

STAGES = ("unripe", "ripe", "overripe", "rotten")


@dataclass
class Det:
    stage: str
    confidence: float
    x: int
    y: int
    w: int
    h: int
    image_width: int
    image_height: int
    class_id: int = 0
    # 회전 사각형 (grasp yaw용). 기본은 축정렬 박스 폴백값.
    center_x: float = 0.0
    center_y: float = 0.0
    rect_w: float = 0.0
    rect_h: float = 0.0
    angle_deg: float = 0.0


def rotated_rect_green(frame: np.ndarray, x1: int, y1: int, x2: int, y2: int):
    """박스 내부 초록 테두리로 minAreaRect → (cx,cy,w,h,angle) 전체프레임 좌표.
    cv2 없거나 초록 미검출이면 None (호출측이 축정렬 박스로 폴백)."""
    try:
        import cv2  # 지연 import (ROS 런타임에 cv2 없어도 노드는 뜬다)
        ya, ya2 = max(0, y1), max(0, y2)
        xa, xa2 = max(0, x1), max(0, x2)
        roi = frame[ya:ya2, xa:xa2]
        if roi.size == 0:
            return None
        # node._to_numpy 는 RGB 를 넘김 → RGB2HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, (35, 60, 40), (90, 255, 255))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        (cx, cy), (w, h), ang = cv2.minAreaRect(max(cnts, key=cv2.contourArea))
        return (cx + xa, cy + ya, w, h, ang)
    except Exception:
        return None


class StubDetector:
    """모델 없이 그럴듯한 랜덤 감지. 카메라/GPU/가중치 없이 전체 흐름 테스트."""

    def __init__(self, classes: Sequence[str] = STAGES) -> None:
        self._classes = list(classes)

    def detect(self, frame: np.ndarray) -> List[Det]:
        h, w = frame.shape[:2]
        if random.random() < 0.2:      # 가끔 아무것도 없음
            return []
        bw, bh = int(w * 0.3), int(h * 0.35)
        stage = random.choice(self._classes)
        bx, by = random.randint(0, w - bw), random.randint(0, h - bh)
        return [Det(
            stage=stage,
            confidence=round(0.80 + random.random() * 0.19, 3),
            x=bx, y=by, w=bw, h=bh, image_width=w, image_height=h,
            class_id=self._classes.index(stage),
            center_x=bx + bw / 2, center_y=by + bh / 2,
            rect_w=float(bw), rect_h=float(bh), angle_deg=0.0,
        )]


class YoloDetector:
    """ultralytics 모델. detect(frame) → Det 리스트."""

    def __init__(self, model_path: str, classes: Sequence[str],
                 conf: float = 0.5, device: str = "cuda:0") -> None:
        from ultralytics import YOLO      # 지연 import
        self._model = YOLO(model_path)
        self._classes = list(classes)
        self._conf = conf
        self._device = device

    def detect(self, frame: np.ndarray) -> List[Det]:
        h, w = frame.shape[:2]
        results = self._model(frame, conf=self._conf, device=self._device, verbose=False)
        dets: List[Det] = []
        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                if cls >= len(self._classes):
                    continue
                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
                rr = rotated_rect_green(frame, x1, y1, x2, y2)
                if rr is None:      # 초록 미검출 → 축정렬 박스 폴백
                    rr = (x1 + (x2 - x1) / 2, y1 + (y2 - y1) / 2,
                          float(x2 - x1), float(y2 - y1), 0.0)
                cxr, cyr, rw, rh, ang = rr
                dets.append(Det(
                    stage=self._classes[cls],
                    confidence=float(box.conf[0]),
                    x=x1, y=y1, w=x2 - x1, h=y2 - y1,
                    image_width=w, image_height=h,
                    class_id=cls,
                    center_x=cxr, center_y=cyr,
                    rect_w=rw, rect_h=rh, angle_deg=ang,
                ))
        return dets


def make_detector(model_path: str, classes: Sequence[str],
                  conf: float, device: str):
    """모델 파일 있으면 Yolo, 없으면 Stub. import 실패해도 Stub로 폴백."""
    if model_path and os.path.exists(model_path):
        try:
            return YoloDetector(model_path, classes, conf, device)
        except Exception as e:  # torch/CUDA 문제 등
            print(f"[perception] YOLO 로드 실패 → Stub 사용: {e}")
    else:
        print(f"[perception] 모델 없음({model_path}) → Stub 사용")
    return StubDetector(classes)
