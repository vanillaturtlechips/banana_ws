#include "doosan_mini_project_moveit_config/mtc_task_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>

namespace mtc = moveit::task_constructor;

namespace doosan_mini_project_moveit_config
{

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("mtc_task_node", options)
{

  // 생성자에서 파라미터가 이미 선언됐는지 확인한 뒤, 선언되지 않은 경우에만 선언
  if (!has_parameter("object_size_x")) {
    declare_parameter("object_size_x", object_size_x_);
  }

  if (!has_parameter("object_size_y")) {
    declare_parameter("object_size_y", object_size_y_);
  }

  if (!has_parameter("object_size_z")) {
    declare_parameter("object_size_z", object_size_z_);
  }

  if (!has_parameter("grasp_angle_delta")) {
    declare_parameter("grasp_angle_delta", grasp_angle_delta_);
  }

  if (!has_parameter("pick_max_ik_solutions")) {
    declare_parameter(
      "pick_max_ik_solutions",
      pick_max_ik_solutions_);
  }

  if (!has_parameter("place_max_ik_solutions")) {
    declare_parameter(
      "place_max_ik_solutions",
      place_max_ik_solutions_);
  }

  if (!has_parameter("ik_min_solution_distance")) {
    declare_parameter(
      "ik_min_solution_distance",
      ik_min_solution_distance_);
  }

  if (!has_parameter("connect_timeout_sec")) {
    declare_parameter(
      "connect_timeout_sec",
      connect_timeout_sec_);
  }

  if (!has_parameter("max_task_solutions")) {
    declare_parameter(
      "max_task_solutions",
      max_task_solutions_);
  }
  get_parameter("object_size_x", object_size_x_);
  get_parameter("object_size_y", object_size_y_);
  get_parameter("object_size_z", object_size_z_);

  get_parameter("grasp_angle_delta", grasp_angle_delta_);
  get_parameter("pick_max_ik_solutions", pick_max_ik_solutions_);
  get_parameter("place_max_ik_solutions", place_max_ik_solutions_);
  get_parameter("ik_min_solution_distance", ik_min_solution_distance_);
  get_parameter("connect_timeout_sec", connect_timeout_sec_);
  get_parameter("max_task_solutions", max_task_solutions_);

  grasp_angle_delta_ = std::max(grasp_angle_delta_, 0.001);
  pick_max_ik_solutions_ = std::max(pick_max_ik_solutions_, 1);
  place_max_ik_solutions_ = std::max(place_max_ik_solutions_, 1);
  ik_min_solution_distance_ = std::max(ik_min_solution_distance_, 0.0);
  connect_timeout_sec_ = std::max(connect_timeout_sec_, 0.1);
  max_task_solutions_ = std::max(max_task_solutions_, 1);

  worker_thread_ = std::thread(&MTCTaskNode::workerLoop, this);

  RCLCPP_INFO(
    get_logger(),
    "MTC worker started: grasp_delta=%.3f, pick_ik=%d, place_ik=%d, "
    "connect_timeout=%.2f, task_solutions=%d",
    grasp_angle_delta_,
    pick_max_ik_solutions_,
    place_max_ik_solutions_,
    connect_timeout_sec_,
    max_task_solutions_);
}

MTCTaskNode::~MTCTaskNode()
{
  stop_requested_.store(true);
  queue_cv_.notify_all();

  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

bool MTCTaskNode::submitJob(const PickPlaceJob& job)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);

  if (busy_.load() || !job_queue_.empty()) {
    return false;
  }

  job_queue_.push_back(job);
  queue_cv_.notify_one();
  return true;
}

void MTCTaskNode::workerLoop()
{
  while (rclcpp::ok() && !stop_requested_.load()) {
    PickPlaceJob job;

    {
      std::unique_lock<std::mutex> lock(queue_mutex_);

      queue_cv_.wait(lock, [this]() {
        return stop_requested_.load() || !job_queue_.empty();
      });

      if (stop_requested_.load()) {
        break;
      }

      job = job_queue_.front();
      job_queue_.pop_front();
      busy_.store(true);
    }

    try {
      processJob(job);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Unhandled worker exception: %s", e.what());
    }

    busy_.store(false);
  }
}

bool MTCTaskNode::processJob(const PickPlaceJob& job)
{
  RCLCPP_INFO(
    get_logger(),
    "Starting task: pick=(%.3f, %.3f, %.3f), class_id=%d",
    job.object_pose.position.x,
    job.object_pose.position.y,
    job.object_pose.position.z,
    job.class_id);

  const auto total_start = std::chrono::steady_clock::now();

  try {
    if (!setupPlanningScene(job)) {
      return false;
    }

    const auto place_pose = selectPlacePose(job.class_id);

    task_ = createTask(job, place_pose);
    task_.init();

  } catch (const mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(get_logger(), "Task initialization failed:\n" << e);
    return false;

  } catch (const moveit::Exception& e) {
    RCLCPP_ERROR(get_logger(), "MoveIt exception: %s", e.what());
    return false;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Task construction exception: %s", e.what());
    return false;
  }

  const auto planning_start = std::chrono::steady_clock::now();

  const moveit::core::MoveItErrorCode plan_result =
    task_.plan(static_cast<std::size_t>(max_task_solutions_));

  const auto planning_end = std::chrono::steady_clock::now();
  const double planning_seconds =
    std::chrono::duration<double>(planning_end - planning_start).count();

  const bool planning_succeeded =
    plan_result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

  RCLCPP_INFO(
    get_logger(),
    "MTC planning finished: success=%s, error_code=%d, elapsed=%.3f sec, solutions=%zu",
    planning_succeeded ? "true" : "false",
    plan_result.val,
    planning_seconds,
    task_.solutions().size());

  if (!planning_succeeded) {
    RCLCPP_ERROR(
      get_logger(),
      "Task planning failed: error_code=%d",
      plan_result.val);
    return false;
  }

  if (task_.solutions().empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "Task planning returned SUCCESS, but no solution was stored");
    return false;
  }

  task_.introspection().publishSolution(*task_.solutions().front());
  task_.introspection().publishTaskDescription();

  const auto execution_start = std::chrono::steady_clock::now();

  const moveit::core::MoveItErrorCode execution_result =
    task_.execute(*task_.solutions().front());

  const auto execution_end = std::chrono::steady_clock::now();
  const double execution_seconds =
    std::chrono::duration<double>(execution_end - execution_start).count();

  RCLCPP_INFO(
    get_logger(),
    "MTC execution finished: elapsed=%.3f sec, error_code=%d",
    execution_seconds,
    execution_result.val);

  if (execution_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(
      get_logger(),
      "Task execution failed: error_code=%d",
      execution_result.val);
    return false;
  }

  const auto total_end = std::chrono::steady_clock::now();
  const double total_seconds =
    std::chrono::duration<double>(total_end - total_start).count();

  RCLCPP_INFO(
    get_logger(),
    "Pick-and-place completed successfully: total_elapsed=%.3f sec",
    total_seconds);

  return true;
}

bool MTCTaskNode::setupPlanningScene(const PickPlaceJob& job)
{
  moveit::planning_interface::PlanningSceneInterface psi;

  psi.removeCollisionObjects({"object"});

  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = job.frame_id;
  object.operation = moveit_msgs::msg::CollisionObject::ADD;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {
    object_size_x_,
    object_size_y_,
    object_size_z_
  };

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(job.object_pose);

  if (!psi.applyCollisionObject(object)) {
    RCLCPP_ERROR(get_logger(), "Failed to apply collision object");
    return false;
  }

  RCLCPP_INFO(
    get_logger(),
    "Collision object created: frame=%s, center=(%.3f, %.3f, %.3f), "
    "quaternion=(%.3f, %.3f, %.3f, %.3f)",
    object.header.frame_id.c_str(),
    job.object_pose.position.x,
    job.object_pose.position.y,
    job.object_pose.position.z,
    job.object_pose.orientation.x,
    job.object_pose.orientation.y,
    job.object_pose.orientation.z,
    job.object_pose.orientation.w);

  return true;
}

geometry_msgs::msg::PoseStamped
MTCTaskNode::selectPlacePose(std::int32_t class_id) const
{
  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = "world";

  target.pose.orientation.x = 0.0;
  target.pose.orientation.y = 0.0;
  target.pose.orientation.z = 0.0;
  target.pose.orientation.w = 1.0;

  target.pose.position.z = object_size_z_ / 2.0;

  switch (class_id) {
    case 0:
      target.pose.position.x = 0.60;
      target.pose.position.y = -0.20;
      break;

    case 1:
      target.pose.position.x = 0.60;
      target.pose.position.y = 0.00;
      break;

    case 2:
      target.pose.position.x = 0.60;
      target.pose.position.y = 0.20;
      break;

    case 3:
      target.pose.position.x = 0.45;
      target.pose.position.y = 0.35;
      break;

    default:
      throw std::invalid_argument("class_id must be 0, 1, 2, or 3");
  }

  return target;
}

mtc::Task MTCTaskNode::createTask(
  const PickPlaceJob&,
  const geometry_msgs::msg::PoseStamped& place_pose)
{
  mtc::Task task;
  task.stages()->setName("class_based_pick_and_place");
  task.loadRobotModel(shared_from_this());

  const std::string arm_group_name = "arm";
  const std::string hand_group_name = "hand";
  const std::string hand_frame = "rh_p12_rn_base";

  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  mtc::Stage* current_state_ptr = nullptr;

  auto current_state =
    std::make_unique<mtc::stages::CurrentState>("Current State");

  current_state_ptr = current_state.get();
  task.add(std::move(current_state));

  auto sampling_planner =
    std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());

  auto interpolation_planner =
    std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner =
    std::make_shared<mtc::solvers::CartesianPath>();

  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.01);

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>(
      "Move Home",
      interpolation_planner);

    stage->setGroup(arm_group_name);
    stage->setGoal("home");
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>(
      "Open Gripper",
      interpolation_planner);

    stage->setGroup(hand_group_name);
    stage->setGoal("gripper_open");
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::Connect>(
      "Connect to Pick",
      mtc::stages::Connect::GroupPlannerVector{
        {arm_group_name, sampling_planner}
      });

    stage->setTimeout(connect_timeout_sec_);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  mtc::Stage* attach_object_stage = nullptr;

  {
    auto pick = std::make_unique<mtc::SerialContainer>("Pick Object");

    task.properties().exposeTo(
      pick->properties(),
      {"eef", "group", "ik_frame"});

    pick->properties().configureInitFrom(
      mtc::Stage::PARENT,
      {"eef", "group", "ik_frame"});

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>(
        "Approach Object",
        cartesian_planner);

      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(
        mtc::Stage::PARENT,
        {"group"});

      stage->setMinMaxDistance(0.10, 0.15);

      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = "world";
      direction.vector.z = -1.0;

      stage->setDirection(direction);
      pick->insert(std::move(stage));
    }

    {
      auto generator =
        std::make_unique<mtc::stages::GenerateGraspPose>(
          "Generate Grasp Pose");

      generator->properties().configureInitFrom(mtc::Stage::PARENT);
      generator->properties().set("marker_ns", "grasp_pose");
      generator->setPreGraspPose("gripper_open");
      generator->setObject("object");
      generator->setAngleDelta(grasp_angle_delta_);
      generator->setMonitoredStage(current_state_ptr);

      Eigen::Isometry3d grasp_frame_transform =
        Eigen::Isometry3d::Identity();

      const Eigen::Quaterniond rotation =
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ());

      grasp_frame_transform.linear() = rotation.toRotationMatrix();
      grasp_frame_transform.translation().z() = 0.145;

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>(
        "Compute Pick IK",
        std::move(generator));

      wrapper->setMaxIKSolutions(pick_max_ik_solutions_);
      wrapper->setMinSolutionDistance(ik_min_solution_distance_);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);

      wrapper->properties().configureInitFrom(
        mtc::Stage::PARENT,
        {"eef", "group"});

      wrapper->properties().configureInitFrom(
        mtc::Stage::INTERFACE,
        {"target_pose"});

      pick->insert(std::move(wrapper));
    }

    {
      auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>(
          "Allow Hand-Object Collision");

      stage->allowCollisions(
        "object",
        task.getRobotModel()
          ->getJointModelGroup(hand_group_name)
          ->getLinkModelNamesWithCollisionGeometry(),
        true);

      pick->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>(
        "Close Gripper",
        interpolation_planner);

      stage->setGroup(hand_group_name);
      stage->setGoal("gripper_close");
      pick->insert(std::move(stage));
    }

    {
      auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>(
          "Attach Object");

      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get();
      pick->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>(
        "Lift Object",
        cartesian_planner);

      stage->properties().configureInitFrom(
        mtc::Stage::PARENT,
        {"group"});

      stage->setMinMaxDistance(0.10, 0.30);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = "world";
      direction.vector.z = 1.0;

      stage->setDirection(direction);
      pick->insert(std::move(stage));
    }

    task.add(std::move(pick));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>(
      "Return Home With Object",
      interpolation_planner);

    stage->setGroup(arm_group_name);
    stage->setGoal("home");
    task.add(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::Connect>(
      "Connect to Place",
      mtc::stages::Connect::GroupPlannerVector{
        {arm_group_name, sampling_planner},
        {hand_group_name, interpolation_planner}
      });

    stage->setTimeout(connect_timeout_sec_);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("Place Object");

    task.properties().exposeTo(
      place->properties(),
      {"eef", "group", "ik_frame"});

    place->properties().configureInitFrom(
      mtc::Stage::PARENT,
      {"eef", "group", "ik_frame"});

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>(
        "Approach Place",
        cartesian_planner);

      stage->properties().set("marker_ns", "approach_place");
      stage->properties().set("link", hand_frame);

      stage->properties().configureInitFrom(
        mtc::Stage::PARENT,
        {"group"});

      stage->setMinMaxDistance(0.10, 0.15);

      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = "world";
      direction.vector.z = -1.0;

      stage->setDirection(direction);
      place->insert(std::move(stage));
    }

    {
      auto generator =
        std::make_unique<mtc::stages::GeneratePlacePose>(
          "Generate Place Pose");

      generator->properties().configureInitFrom(mtc::Stage::PARENT);
      generator->properties().set("marker_ns", "place_pose");
      generator->setObject("object");
      generator->setPose(place_pose);
      generator->setMonitoredStage(attach_object_stage);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>(
        "Compute Place IK",
        std::move(generator));

      wrapper->setMaxIKSolutions(place_max_ik_solutions_);
      wrapper->setMinSolutionDistance(ik_min_solution_distance_);
      wrapper->setIKFrame("object");

      wrapper->properties().configureInitFrom(
        mtc::Stage::PARENT,
        {"eef", "group"});

      wrapper->properties().configureInitFrom(
        mtc::Stage::INTERFACE,
        {"target_pose"});

      place->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>(
        "Open Gripper at Place",
        interpolation_planner);

      stage->setGroup(hand_group_name);
      stage->setGoal("gripper_open");
      place->insert(std::move(stage));
    }

    {
      auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>(
          "Forbid Hand-Object Collision");

      stage->allowCollisions(
        "object",
        task.getRobotModel()
          ->getJointModelGroup(hand_group_name)
          ->getLinkModelNamesWithCollisionGeometry(),
        false);

      place->insert(std::move(stage));
    }

    {
      auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>(
          "Detach Object");

      stage->detachObject("object", hand_frame);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>(
        "Retreat From Place",
        cartesian_planner);

      stage->properties().configureInitFrom(
        mtc::Stage::PARENT,
        {"group"});

      stage->setMinMaxDistance(0.12, 0.35);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat_place");

      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = "world";
      direction.vector.z = 1.0;

      stage->setDirection(direction);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>(
      "Final Home",
      interpolation_planner);

    stage->setGroup(arm_group_name);
    stage->setGoal("home");
    task.add(std::move(stage));
  }

  return task;
}

}  // namespace doosan_mini_project_moveit_config