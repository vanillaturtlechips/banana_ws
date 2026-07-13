# models/

학습된 YOLO 가중치를 여기에 둡니다.

- `best.pt` — ultralytics 학습 결과 (또는 `best.engine` = TensorRT)
- **⚠️ git에 커밋하지 마세요** (수십~수백 MB). `.gitignore`가 `*.pt/*.engine`를 무시함.
- 팀원은 학습 후 이 폴더에 직접 넣거나, 공유 스토리지에서 받으세요.

파일이 없으면 노드는 자동으로 **StubDetector**(랜덤 감지)로 폴백해서
GPU·가중치 없이도 파이프라인이 돕니다.

## 5080(Blackwell) 학습/추론 메모
```bash
# torch를 CUDA 12.8 빌드로 먼저 설치 (sm_120)
pip install torch --index-url https://download.pytorch.org/whl/cu128
pip install ultralytics

# 학습
yolo train model=yolov8n.pt data=banana.yaml epochs=100 imgsz=640

# (선택) TensorRT로 export — Python 유지하면서 속도 ↑
yolo export model=best.pt format=engine
```
