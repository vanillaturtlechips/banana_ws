# 🍌 BananaSort 설치 가이드 (처음부터 재현)

바나나 익음 분류 로봇 파이프라인을 **빈 머신에서 끝까지** 세우는 가이드입니다.
자유 한국어 명령 → LLM → 감지(YOLO) → 에이전트(선택+정책) → MoveIt 픽앤플레이스 + 실시간 웹까지.

```
[웹/채팅] → LLM(Ollama) → SortCommand → perception(YOLO) → agent(선택+게이트)
   → /detection → MoveIt(MTC) → 로봇팔
        └ 카메라 영상(WebRTC) · 감지 상태 → 웹
```

---

## 0. 시스템 요구사항

| 항목 | 값 |
|---|---|
| OS | Ubuntu 24.04 (Noble) |
| ROS | ROS 2 **Jazzy** |
| GPU | NVIDIA (Blackwell/**RTX 5080**에서 검증, VRAM 8GB+) + 드라이버 |
| 카메라 | Intel RealSense **D455/D455F** (USB 3.x) |
| 로봇(선택) | Doosan e0509 (없으면 fake/sim으로 대체) |

> GPU가 Blackwell(sm_120)이면 torch는 **CUDA 12.8+ 빌드**가 필수입니다.

---

## 1. ROS 2 Jazzy

[공식 문서](https://docs.ros.org/en/jazzy/Installation.html) 대로 `ros-jazzy-desktop` 설치 후:
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source /opt/ros/jazzy/setup.bash
sudo apt install -y python3-colcon-common-extensions
```

## 2. 워크스페이스 클론

```bash
git clone <banana_ws_repo_URL> ~/banana_ws
# (선택) 실물 두산 그리퍼용 패키지
cd ~/banana_ws/src && git clone https://github.com/pinklab-art/dsr_study.git
```
> `doosan_mini_project_*`(MoveIt/MTC) 패키지는 이미 레포에 포함돼 있습니다.

## 3. ROS apt 의존성 (한 줄로)

```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-realsense2-camera \
  ros-jazzy-moveit \
  ros-jazzy-moveit-task-constructor-core \
  ros-jazzy-moveit-task-constructor-msgs \
  ros-jazzy-moveit-ros-planning-interface \
  ros-jazzy-ros2-controllers
```
- `realsense2_camera` — 카메라 ROS 드라이버 (pyrealsense2 SDK와 별개!)
- `moveit` + `moveit-task-constructor-*` — 픽앤플레이스 계획(MTC)
- `ros2-controllers` — `joint_trajectory_controller`(arm/hand 실행)

> ⚠️ 두산 로봇 description(`dsr_description2`)은 **불필요** — e0509 URDF가 config에 내장돼 있습니다.

## 4. Python venv — YOLO/카메라 (핵심)

perception 노드가 torch/ultralytics/pyrealsense2를 쓰는데, ROS 파이썬엔 없어서 별도 venv를 만들고
**실행 시 활성화**하면 런치가 그 site-packages를 자동 주입합니다.

```bash
python3 -m venv ~/venv/yolov3
source ~/venv/yolov3/bin/activate

# Blackwell(RTX 5080)은 CUDA 12.8+ 빌드 필수
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
pip install ultralytics pyrealsense2 opencv-python numpy

# 브리지(웹 게이트웨이) 파이썬 의존
pip install -r ~/banana_ws/src/banana_bridge/requirements.txt
```
확인:
```bash
python -c "import torch,ultralytics,pyrealsense2,cv2; print('torch', torch.__version__, 'cuda', torch.cuda.is_available())"
```

## 5. Ollama — 로컬 LLM (자유 한국어 명령)

```bash
curl -fsSL https://ollama.com/install.sh | sh   # systemd 서비스로 11434 자동 기동
ollama pull qwen2.5:7b                           # 한국어+JSON 명령 파싱 (~4.7GB)
ollama list                                      # qwen2.5:7b 보이면 OK
```
> 키 없이 동작. `llm_provider:=fake`면 Ollama 없이 키워드 파서로도 기본 명령은 됩니다.

## 6. 프론트엔드 (bun)

```bash
curl -fsSL https://bun.sh/install | bash && source ~/.bashrc
git clone <frontend_repo_URL> ~/banana-frontend
cd ~/banana-frontend && bun install
```

## 7. 모델 가중치 (best.pt)

`*.pt`는 gitignore 대상이라 레포에 없습니다. 학습 결과를 여기에 두세요:
```
~/banana_ws/src/banana_perception/models/best.pt
```
- 학습법: 데이터셋의 `scripts/train.py`(yolov8n, imgsz 640) 참고. 클래스 순서는 반드시
  `[unripe, ripe, overripe, rotten]` (perception config와 일치).
- 없으면 perception이 **StubDetector(랜덤)**로 폴백합니다.

## 8. 빌드

```bash
cd ~/banana_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

---

## 9. 실행 (시뮬 — 실물 로봇 없이 전 구간)

> **매번 한 세트만.** 재시작 전 반드시 아래로 잔존 프로세스 정리:
> ```bash
> pkill -9 -f "ros2 launch"; pkill -9 -f realsense2_camera_node; \
> pkill -9 -f "banana_perception/lib"; pkill -9 -f "uvicorn banana_bridge"; \
> pkill -9 -f pick_place_system; pkill -9 -f move_group; sleep 3
> ```

**터미널 1 — MoveIt (fake 로봇 + rviz + MTC 실행 capability)**
```bash
source /opt/ros/jazzy/setup.bash && source ~/banana_ws/install/setup.bash
ros2 launch doosan_mini_project_moveit_config mtc_demo.launch.py
```
> ⚠️ **`demo.launch.py`가 아니라 `mtc_demo.launch.py`!** 후자에만
> `move_group/ExecuteTaskSolutionCapability`가 있어서 MTC가 계획한 solution을
> **실제로 실행**할 수 있습니다. demo.launch.py는 계획돼도 실행이 안 됩니다.
**터미널 2 — 픽앤플레이스 (`/detection` 구독)**
```bash
source /opt/ros/jazzy/setup.bash && source ~/banana_ws/install/setup.bash
ros2 launch doosan_mini_project_moveit_config pick_and_place_system.launch.py
```
**터미널 3 — 카메라 + perception + agent + bridge + LLM**
```bash
source ~/venv/yolov3/bin/activate && source ~/banana_ws/install/setup.bash
ros2 launch banana_bridge l2_integration.launch.py llm_provider:=local
```
**터미널 4 — 웹 프론트**
```bash
export PATH="$HOME/.bun/bin:$PATH" && cd ~/banana-frontend && bun --bun run dev
# node 18이면 vite가 안 떠서 --bun 필수. 표시된 주소를 브라우저로.
```

### 동작 확인
- 웹에 카메라 영상 + "연결됨", 주사위 놓으면 감지 박스
- 채팅: **"가장 왼쪽 거 집어줘"** / **"익은 것만 분류해줘"** / **"오른쪽 뭐야?"**
- 검증 토픽:
  ```bash
  ros2 topic echo /banana/detection --once   # has_depth: true 확인
  ros2 topic echo /banana/agent_status        # 선택+게이트 결정
  ros2 topic echo /detection                  # MoveIt으로 나가는 집기 대상
  ```
- MoveIt 없이 연결만 볼 땐 터미널 2 대신:
  `python ~/banana_ws/tools/mock_moveit.py`

---

## 10. 실물 로봇 (Doosan) 노트

시뮬(`demo.launch.py`)은 fake 컨트롤러라 로봇이 실제로 안 움직입니다. 실물은:
1. **hand-eye 캘리브레이션** — `perception.launch.py`의 placeholder 정적 TF
   (`base_link → camera_link`)를 easy_handeye2 등으로 구한 실측값으로 교체.
2. **Doosan 드라이버**로 실팔 연결 (별도 네트워크/IP) + `dsr_gripper`.
3. **MTC 계획 튜닝** — grasp 접근/IK/타임아웃 파라미터
   (`pick_and_place_system.launch.py` 참고).
> 인터페이스는 이미 맞물림: agent가 `/detection`(`banana_command/Detection`: point_xyz·angle_deg·class_id·has_depth)을 발행 → `pick_place_command_node`가 구독.

---

## 11. 트러블슈팅 (실제로 겪은 것들)

| 증상 | 원인 / 해결 |
|---|---|
| 감지 신뢰도 0.5로 뭉갬 | RGB/BGR 채널 뒤바뀜. detector가 YOLO에 BGR 넣도록 이미 수정됨 (재빌드) |
| 빈 배경에도 감지 | 초록테두리 게이트가 걸러냄 (주사위 초록 테두리 필요) |
| 카메라 `Depth stream failure`/`Frames didn't arrive` | 장치 wedge. 런치에 `initial_reset` 포함. 그래도면 USB 재연결 |
| `Device or resource busy` | 이전 launch 잔존. 위 pkill 정리 |
| LLM이 JSON 원문을 채팅에 노출 | fake→local 전환 후 재시작 (프롬프트는 시작 시 로드) |
| WebRTC ICE failed | 양쪽 STUN 지정됨. 브리지 재빌드+강력새로고침 |
| 계획은 되는데 로봇이 실행 안 됨 | move_group에 `ExecuteTaskSolutionCapability` 필요 → `mtc_demo.launch.py` 사용(`demo.launch.py` 말고) |
| MoveIt `MTC planning ... solutions=0` | 캘리브 미완/작업범위 밖. 10번(실물 노트) 참고 |
| perception이 `detector=StubDetector` | best.pt 없음 or venv 미활성. 7번/4번 확인 |

---

문의: 파이프라인 계약은 `src/banana_command/INTERFACE.md` 참고.
