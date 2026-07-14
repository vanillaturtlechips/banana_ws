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
};

}  // namespace doosan_mini_project_moveit_config