#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <banana_command/msg/detection.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>

#include "doosan_mini_project_moveit_config/mtc_task_node.hpp"

namespace doosan_mini_project_moveit_config
{

class PickPlaceCommandNode : public rclcpp::Node
{
public:
  PickPlaceCommandNode(
    const std::shared_ptr<MTCTaskNode>& mtc_node,
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("pick_place_command_node", options), mtc_node_(mtc_node)
  {
    subscription_ = create_subscription<banana_command::msg::Detection>(
      "/detection",
      rclcpp::SensorDataQoS(),
      std::bind(&PickPlaceCommandNode::detectionCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Waiting for /detection [banana_command/msg/Detection]: "
      "point_x/y/z, angle_deg, class_id(0..3)");
  }

private:
  static geometry_msgs::msg::Quaternion yawToQuaternion(const double yaw_rad)
  {
    geometry_msgs::msg::Quaternion quaternion;
    const double half_yaw = yaw_rad * 0.5;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = std::sin(half_yaw);
    quaternion.w = std::cos(half_yaw);
    return quaternion;
  }

  void detectionCallback(const banana_command::msg::Detection::SharedPtr msg)
  {
    if (!msg->has_depth) {
      RCLCPP_WARN(
        get_logger(),
        "Detection rejected: has_depth=false, stage=%s, class_id=%u",
        msg->stage.c_str(), static_cast<unsigned int>(msg->class_id));
      return;
    }

    if (msg->class_id > 3U) {
      RCLCPP_ERROR(
        get_logger(),
        "Detection rejected: unsupported class_id=%u (allowed: 0..3)",
        static_cast<unsigned int>(msg->class_id));
      return;
    }

    if (!std::isfinite(msg->point_x) ||
        !std::isfinite(msg->point_y) ||
        !std::isfinite(msg->point_z) ||
        !std::isfinite(msg->angle_deg))
    {
      RCLCPP_ERROR(get_logger(), "Detection rejected: position or angle contains NaN/Inf");
      return;
    }

    PickPlaceJob job;
    job.frame_id = msg->header.frame_id.empty() ? "world" : msg->header.frame_id;
    job.class_id = static_cast<std::int32_t>(msg->class_id);
    job.confidence = msg->confidence;

    job.object_pose.position.x = static_cast<double>(msg->point_x);
    job.object_pose.position.y = static_cast<double>(msg->point_y);
    job.object_pose.position.z = static_cast<double>(msg->point_z);

    // OpenCV minAreaRect angle_deg를 메시지 frame의 +Z축 yaw로 직접 해석한다.
    const double yaw_rad =
      static_cast<double>(msg->angle_deg) * 3.14159265358979323846 / 180.0;
    job.object_pose.orientation = yawToQuaternion(yaw_rad);

    if (!mtc_node_->submitJob(job)) {
      RCLCPP_WARN(
        get_logger(),
        "MTC busy: detection rejected, class_id=%u, frame=%s, "
        "point=(%.3f, %.3f, %.3f), yaw=%.2f deg",
        static_cast<unsigned int>(msg->class_id), job.frame_id.c_str(),
        msg->point_x, msg->point_y, msg->point_z, msg->angle_deg);
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Detection accepted: stage=%s, class_id=%u, confidence=%.3f, frame=%s, "
      "point=(%.3f, %.3f, %.3f), yaw=%.2f deg",
      msg->stage.c_str(), static_cast<unsigned int>(msg->class_id), msg->confidence,
      job.frame_id.c_str(), msg->point_x, msg->point_y, msg->point_z, msg->angle_deg);
  }

  std::shared_ptr<MTCTaskNode> mtc_node_;
  rclcpp::Subscription<banana_command::msg::Detection>::SharedPtr subscription_;
};

}  // namespace doosan_mini_project_moveit_config

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions base_options;
  base_options.automatically_declare_parameters_from_overrides(true);

  // launch 파일에서 executable 전체에 __node remap이 걸려도 내부 두 노드가
  // 동일한 이름을 갖지 않도록 각각의 local remap을 명시한다.
  auto mtc_options = base_options;
  mtc_options.arguments({"--ros-args", "-r", "__node:=mtc_task_node"});

  auto command_options = base_options;
  command_options.arguments({"--ros-args", "-r", "__node:=pick_place_command_node"});

  auto mtc_node =
    std::make_shared<doosan_mini_project_moveit_config::MTCTaskNode>(mtc_options);
  auto command_node =
    std::make_shared<doosan_mini_project_moveit_config::PickPlaceCommandNode>(
      mtc_node, command_options);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(mtc_node);
  executor.add_node(command_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}