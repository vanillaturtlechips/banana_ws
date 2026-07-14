#!/usr/bin/env python3
"""Mock MoveIt — 실제 MoveIt/MTC/두산 스택이 없는 머신에서 연동 검증용.

MoveIt팀 pick_place_command_node.cpp 의 검증 로직을 그대로 흉내내어
/detection(banana_command/Detection)을 받아:
  - REJECTED: has_depth=false / class_id>3 / point·angle NaN·Inf
  - ACCEPTED: 집기 실행을 로그로 흉내 (실제 로봇 대신 스탠드인)

실 MoveIt 머신에선 이 노드 대신 pick_and_place_system.launch.py 를 띄운다.
실행: python3 tools/mock_moveit.py  (install source 후)
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from banana_command.msg import Detection


class MockMoveIt(Node):
    def __init__(self) -> None:
        super().__init__("mock_moveit")
        # 실 노드와 동일 QoS(SensorData) + 토픽(/detection)
        self.create_subscription(Detection, "/detection", self._cb, qos_profile_sensor_data)
        self.get_logger().info("mock_moveit: /detection 대기 중 (실 MoveIt 대체)")

    def _cb(self, m: Detection) -> None:
        if not m.has_depth:
            self.get_logger().warn(f"REJECTED: has_depth=false (stage={m.stage})")
            return
        if m.class_id > 3:
            self.get_logger().error(f"REJECTED: class_id={m.class_id} (allowed 0..3)")
            return
        if not all(math.isfinite(v) for v in (m.point_x, m.point_y, m.point_z, m.angle_deg)):
            self.get_logger().error("REJECTED: position/angle NaN·Inf")
            return
        self.get_logger().info(
            f"✅ ACCEPTED → 집기 실행(흉내): class_id={m.class_id}({m.stage}) "
            f"conf={m.confidence:.2f} pos=({m.point_x:.3f},{m.point_y:.3f},{m.point_z:.3f}) "
            f"yaw={m.angle_deg:.1f}° frame={m.header.frame_id or 'world'}")


def main() -> None:
    rclpy.init()
    node = MockMoveIt()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
