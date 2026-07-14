"""에이전트 노드 — 명령 + 감지 → 선택 + 게이트 → pick_target + 상태.

IN : /banana/detections (DetectionArray), /banana/command (SortCommand)
OUT: /banana/pick_target (PoseStamped → MoveIt),
     /banana/agent_status (std_msgs/String, JSON → 브리지/웹)

셀렉터/게이트 순수 로직은 agent.py. 여기선 ROS 배선만.
"""
from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from banana_command.msg import SortCommand, Detection, DetectionArray

from .agent import select_object, check_feasibility, PICKABLE


class AgentNode(Node):
    def __init__(self) -> None:
        super().__init__("banana_agent")
        p = self.declare_parameter
        dets_topic = p("detections_topic", "/banana/detections").value
        cmd_topic = p("command_topic", "/banana/command").value
        self._target_topic = p("pick_target_topic", "/banana/pick_target").value
        self._status_topic = p("agent_status_topic", "/banana/agent_status").value
        # MoveIt(pick_place_command_node)이 구독하는 토픽 — 선택된 Detection 발행
        self._moveit_topic = p("moveit_detection_topic", "/detection").value

        self._dets: list = []
        self.create_subscription(DetectionArray, dets_topic, self._on_dets, 10)
        self.create_subscription(SortCommand, cmd_topic, self._on_cmd, 10)
        self._pose_pub = self.create_publisher(PoseStamped, self._target_topic, 10)
        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._det_pub = self.create_publisher(Detection, self._moveit_topic, 10)
        self.get_logger().info("banana_agent 시작 (command + detections → 선택+게이트)")

    def _on_dets(self, msg: DetectionArray) -> None:
        self._dets = list(msg.detections)

    def _on_cmd(self, msg: SortCommand) -> None:
        action = msg.action
        try:
            params = json.loads(msg.params_json) if msg.params_json else {}
        except Exception:
            params = {}
        by = params.get("by")

        if action in ("stop", "rescan"):
            self._emit(action, True, None, None, "", by)
            return

        # 대상 선택: 공간(by) > 특정 stage > 전체 pickable 최고신뢰
        if by:
            target = select_object(self._dets, by)
        elif list(msg.target_stages):
            target = select_object(self._dets, "stage", stage=msg.target_stages[0])
        else:
            pickable = [d for d in self._dets if d.stage in PICKABLE]
            target = select_object(pickable, "stage")

        ok, reason, dest = check_feasibility(target)
        self._emit(action, ok, target, dest, reason, by)
        if ok and target is not None:
            # 선택+게이트 통과한 Detection을 MoveIt(pick_place)로 → 실제 집기
            self._det_pub.publish(target)
            if getattr(target, "has_pose", False):
                self._pose_pub.publish(target.grasp_pose)

    def _emit(self, action, ok, target, dest, reason, by) -> None:
        status = {
            "action": action, "ok": ok, "by": by,
            "stage": target.stage if target else None,
            "confidence": round(float(target.confidence), 3) if target else None,
            "bin": dest, "reason": reason,
        }
        m = String()
        m.data = json.dumps(status, ensure_ascii=False)
        self._status_pub.publish(m)
        self.get_logger().info(f"명령 처리: {status}")


def main() -> None:
    rclpy.init()
    node = AgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
