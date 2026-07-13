# YOLO 감지 결과 하나. perception → (aggregator/bridge)로 발행.
# 픽셀 bbox + 이미지 크기를 함께 담아 소비자가 %(프론트)든 픽셀(deproject)이든 변환 가능.

std_msgs/Header header

string  stage         # unripe | ripe | overripe | rotten
float32 confidence    # 0..1

# 픽셀 단위 bounding box (좌상단 x,y + 너비/높이)
int32   x
int32   y
int32   w
int32   h

# 원본 프레임 크기 (%/정규화 변환용)
int32   image_width
int32   image_height
