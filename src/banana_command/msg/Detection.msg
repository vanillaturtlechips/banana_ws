# YOLO 감지 결과 하나. perception → (aggregator/bridge)로 발행.
# 픽셀 bbox + 이미지 크기를 함께 담아 소비자가 %(프론트)든 픽셀(deproject)이든 변환 가능.

std_msgs/Header header

string  stage         # unripe | ripe | overripe | rotten
uint8   class_id      # 0=unripe 1=ripe 2=overripe 3=rotten (canonical, classes 순서)
float32 confidence    # 0..1

# 축정렬 bounding box (좌상단 x,y + 너비/높이, 픽셀) — 프론트 %/기존 호환
int32   x
int32   y
int32   w
int32   h

# 원본 프레임 크기 (%/정규화 변환용)
int32   image_width
int32   image_height

# 회전 사각형 (grasp yaw용 — 초록 테두리 cv2.minAreaRect 결과, 픽셀)
# 초록 미검출/Stub이면 축정렬 박스로 폴백 (center=박스중심, w/h=박스, angle=0)
float32 center_x      # 회전 사각형 중심 x (픽셀)
float32 center_y      # 회전 사각형 중심 y (픽셀)
float32 rect_w        # 회전 사각형 너비 (픽셀)
float32 rect_h        # 회전 사각형 높이 (픽셀)
float32 angle_deg     # 이미지 평면 회전각 (OpenCV minAreaRect 규약, -90..0)

# --- 3D (카메라 광학 프레임, 미터). 뎁스 있을 때만 채움 ---
bool    has_depth     # 중심 픽셀 뎁스 유효 여부
float32 center_z      # 중심 픽셀 깊이 (m)
float32 point_x       # deproject X (m, camera_optical_frame)
float32 point_y       # deproject Y (m)
float32 point_z       # deproject Z (= center_z)

# --- base_link 기준 집기 자세. hand-eye TF 성공 시만 채움 ---
bool    has_pose      # grasp_pose 유효 여부
geometry_msgs/PoseStamped grasp_pose   # 위치=deproject점, yaw=angle_deg 매핑
