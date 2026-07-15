#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/task_constructor/task.h>
#include <rclcpp/rclcpp.hpp>

namespace doosan_mini_project_moveit_config
{

struct PickPlaceJob
{
  geometry_msgs::msg::Pose object_pose;
  std::string frame_id{"world"};
  std::int32_t class_id{0};
  float confidence{0.0F};
};

class MTCTaskNode : public rclcpp::Node
{
public:
  explicit MTCTaskNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~MTCTaskNode() override;

  // 외부 구독 노드의 콜백에서 호출한다.
  // 이미 작업 중이거나 대기 작업이 있으면 false를 반환하여 중복 실행을 막는다.
  bool submitJob(const PickPlaceJob& job);

private:
  void workerLoop();
  bool processJob(const PickPlaceJob& job);
  bool setupPlanningScene(const PickPlaceJob& job);
  geometry_msgs::msg::PoseStamped selectPlacePose(std::int32_t class_id) const;
  moveit::task_constructor::Task createTask(
    const PickPlaceJob& job,
    const geometry_msgs::msg::PoseStamped& place_pose);

  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<PickPlaceJob> job_queue_;
  std::thread worker_thread_;
  std::atomic_bool stop_requested_{false};
  std::atomic_bool busy_{false};

  moveit::task_constructor::Task task_;

  // 원본 BOX 크기
  double object_size_x_{0.075};
  double object_size_y_{0.075};
  double object_size_z_{0.080};

  // MTC 탐색량 제한 기본값
  // Detection.angle_deg를 사용하므로 grasp 회전 후보는 기본적으로 1개만 생성한다.
  double grasp_angle_delta_{6.283185307179586};
  int pick_max_ik_solutions_{1};
  int place_max_ik_solutions_{1};
  double ik_min_solution_distance_{0.1};
  double connect_timeout_sec_{2.0};
  int max_task_solutions_{1};

  // grasp 접근 기울기(도). 0=순수 수직(top-down). 카메라 시야가 팔의 수직-잡기
  // 범위(x≈0.65)보다 멀리(x≈0.74) 있어, 먼 물체는 수직으론 IK 없음 → 기울여야 닿음.
  // yaw 스윕과 결합해 사방 기울기 생성 → TRAC-IK가 base쪽 기운 해를 찾음. 6면체라 무방.
  double grasp_pitch_deg_{30.0};

  // 분류 정책상 보관함 2개 (world 기준, 실물 보관함 위치 재서 파라미터로 교체).
  //   bin1: ripe(1)+overripe(2) 적당·너무익음
  //   bin2: rotten(3) 썩음(쓰레기통)
  //   unripe(0)은 에이전트가 애초에 안 집으므로 여기 오지 않음
  double bin1_x_{0.60};
  double bin1_y_{0.00};
  double bin2_x_{0.45};
  double bin2_y_{0.35};

  // 🧱 안전: 테이블/보관함 충돌체. 실물 위치 확정 전엔 placeholder라 기본 OFF
  //    (켜면 잘못된 위치가 계획을 막을 수 있음). 실측 좌표 넣고 add_collision_scene:=true.
  bool add_collision_scene_{false};
  double table_z_{0.0};        // 테이블 윗면 높이(world z)
  double bin_wall_height_{0.10};
  double bin_size_{0.15};      // 보관함 한 변(정사각 근사)

  void addSafetySurfaces();
};

}  // namespace doosan_mini_project_moveit_config