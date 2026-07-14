#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif







    /****************************************************
  ---- *               Initialize                *
     ***************************************************/


static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_machine_tending");
namespace mtc = moveit::task_constructor;
double g_object_depth = 0.0;


enum workpiece_type
{
  BOX,
  CYLINDER,
  SPHERE
};  workpiece_type wp;


// Define rclcpp Node

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene(workpiece_type wp);

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task createTask_unload();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}







    /****************************************************
  ---- *              setup Planning Scene                *
     ***************************************************/
void MTCTaskNode::setupPlanningScene(workpiece_type wp)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);


  double object_depth;

  if (wp == BOX)
  {
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    object.primitives[0].dimensions = {0.075, 0.075, 0.08 }; //

    object_depth = object.primitives[0].dimensions[2];
    g_object_depth = object_depth;
  }
  else if (wp == CYLINDER)
  {
    // object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    // object.primitives[0].dimensions = { 0.1, 0.02 };

    // object_depth = object.primitives[0].dimensions[0];
    // g_object_depth = object_depth;
  }
  else if (wp == SPHERE)
  {

    // // ~~~ //
    // object_depth = 0;
  }




  geometry_msgs::msg::Pose picking_position;

  picking_position.position.x = 0.5;
  picking_position.position.y = 0.2;
  picking_position.position.z = 0.0;
  picking_position.orientation.w = 1.0;  // 기본 자세




  picking_position.position.z = picking_position.position.z + (object_depth / 2);

  object.pose = picking_position;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}





    /****************************************************
  ---- *                 doTask               *
     ***************************************************/
void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());
  task_.introspection().publishTaskDescription();

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}










    /****************************************************
  ---- *                 createTask - load               *
     ***************************************************/

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  // 태스크 이름 설정
  task.stages()->setName("load");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "rh_p12_rn_base";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("home_start");
  current_state_ptr = stage_state_current.get();
  current_state_ptr->properties().set("comment", "(파지 준비 자세) feeder 위의 소재를 파지하기 위한 준비 자세. 신속하고 효율적인 이동을 위해 joint_space_move를 수행함.");
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);



  // 랙 홈 이동 //
  auto stage_home_pos =
    std::make_unique<mtc::stages::MoveTo>("Home", interpolation_planner);
  // 객체 속성 설정
  stage_home_pos->setGroup(arm_group_name);
  stage_home_pos->setGoal("home");
  stage_home_pos->properties().set("comment", "(파지 준비 자세) feeder 위의 소재를 파지하기 위한 준비 자세. 신속하고 효율적인 이동을 위해 joint_space_move를 수행함.");
  // task add
  task.add(std::move(stage_home_pos));




  // 그리퍼 오픈 //
  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("Gripper Open", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("gripper_open");
  stage_open_hand->properties().set("comment", "(파지 준비 자세) feeder 위의 소재를 파지하기 위한 준비 자세. 신속하고 효율적인 이동을 위해 joint_space_move를 수행함.");
  task.add(std::move(stage_open_hand));



  // 홈 -> 접근점 경로 생성 (connect) //
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
    "Approach",
    mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  stage_move_to_pick->properties().set("comment", "(파지 경유점) 소재를 파지하기 위한 사전 준비 자세. 신속하고 효율적인 이동을 위해 joint_space_move를 수행함.");
  task.add(std::move(stage_move_to_pick));

  mtc::Stage* attach_object_stage =
    nullptr;  // Forward attach_object_stage to place pose generator

    // This is an example of SerialContainer usage. It's not strictly needed here.
  // In fact, `task` itself is a SerialContainer by default.

  // 시리얼 컨테이너 생성 (pick object) //
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("Pick Task");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    // clang-format off
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    // clang-format on

    {
      // clang-format off

      // 파지 접근점 //
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("Linear Approach", cartesian_planner);
      // clang-format on
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->properties().set("comment", "(파지 경유점) 소재를 파지하기 위해 직선 하강.");
      stage->setMinMaxDistance(0.1, 0.15);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    /****************************************************
  ---- *               Generate Grasp Pose                *
     ***************************************************/
    {
      // Sample grasp pose
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("Pick");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->properties().set("comment", "(파지점) 그리퍼 핑거로 grasp하여 소재를 파지할때까지 대기");
      stage->setPreGraspPose("gripper_open");
      stage->setObject("object");
      stage->setAngleDelta(M_PI / 2);
      stage->setMonitoredStage(current_state_ptr);  // Hook into current state

      // This is the transform from the object frame to the end-effector frame    
      // Grasp 포즈의 방향을 설정

      Eigen::Isometry3d grasp_frame_transform;


      // Z축 90
      // Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()));


      // Y축 180 -> Z축 90
      Eigen::Quaterniond q =
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.toRotationMatrix();


      // Eigen::Quaterniond q =
      // Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
      // Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
      // grasp_frame_transform.linear() = q.matrix();

      // Eigen::Quaterniond q = Eigen::Quaterniond::Identity();  // (0,0,0,1)
      // grasp_frame_transform.linear() = q.toRotationMatrix();

      double gripper_center_to_tcp = 0.145;
      //double tcp_to_object_center = 0.120853;


      grasp_frame_transform.translation().z() = gripper_center_to_tcp;

      // Compute IK
      // clang-format off
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("ComputeIK to Pick", std::move(stage));
      // clang-format on
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      wrapper->properties().set("comment", "(파지점) 그리퍼 핑거로 grasp하여 소재를 파지할때까지 대기");
      grasp->insert(std::move(wrapper));
    }

    {
      // clang-format off
      // allow collision (hand,object)
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("Allow Collision");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             true);
      stage->properties().set("comment", "(파지점) 그리퍼 핑거와 물체의 충돌을 무시");
      // clang-format on
      grasp->insert(std::move(stage));
    }

    {
      // close hand
      auto stage = std::make_unique<mtc::stages::MoveTo>("Grasp", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("gripper_close");
      stage->properties().set("comment", "(파지점) 그리퍼 핑거로 grasp하여 소재를 파지할때까지 대기");
      grasp->insert(std::move(stage));
    }

    {
      // attach object
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("Object Link Attach");
      stage->attachObject("object", hand_frame);
      stage->properties().set("comment", "(파지점) 물체와 그리퍼 핑거의 링크를 연결");
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    {
      // clang-format off
      // lift object
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("Lift Object", cartesian_planner);
      // clang-format on
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->properties().set("comment", "(파지 들어올림 지점) 소재를 파지 후 들어올림을 위해 직선 상승 이동.");
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    // {
    //   // clang-format off
    //   // lift object
    //   auto stage =
    //       std::make_unique<mtc::stages::MoveRelative>("Retreat from Lift", cartesian_planner);
    //   // clang-format on
    //   stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    //   stage->properties().set("comment", "(파지 후퇴점) 소재를 파지 후 후퇴를 위해 직선 후방 이동");
    //   stage->setMinMaxDistance(0.1, 0.3);
    //   stage->setIKFrame(hand_frame);
    //   stage->properties().set("marker_ns", "lift_object");

    //   // Set upward direction
    //   geometry_msgs::msg::Vector3Stamped vec;
    //   vec.header.frame_id = "world";
    //   vec.vector.x = -1.0;
    //   stage->setDirection(vec);
    //   grasp->insert(std::move(stage));
    // }


    task.add(std::move(grasp));
  }

    // Home Pos //
    auto stage_home_pos2 =
      std::make_unique<mtc::stages::MoveTo>("Return to Home", interpolation_planner);
    // 객체 속성 설정
    stage_home_pos2->setGroup(arm_group_name);
    stage_home_pos2->setGoal("home");
    stage_home_pos2->properties().set("comment", "(파지 복귀 자세) 파지 후 복귀 자세. joint_space_move.");
    // task add
    task.add(std::move(stage_home_pos2));


  {
    // clang-format off
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "Approach",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                  { hand_group_name, interpolation_planner } });
    // clang-format on
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    stage_move_to_place->properties().set("comment", "(배치 경유점) 소재를 배치하기 위한 사전 접근 자세. joint_space_move.");

    task.add(std::move(stage_move_to_place));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    // clang-format off
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    // clang-format on

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("Linear Approach", cartesian_planner);
      // clang-format on
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->properties().set("comment", "(배치 경유점) 소재를 안전하고 정확하게 배치하기 위해 직선 하강.");
      stage->setMinMaxDistance(0.1, 0.15);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    /****************************************************
  ---- *               Generate Place Pose                *
     ***************************************************/

    {
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("Place Object");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->properties().set("comment", "(배치점) 그리퍼 핑거를 release하여 소재를 배치할때까지 대기");
      stage->setObject("object");

      geometry_msgs::msg::PoseStamped target_pose_msg;
      // target_pose_msg.header.frame_id = "object";
      // target_pose_msg.pose.position.y = 0.5;
      // target_pose_msg.pose.orientation.w = 1.0;


      target_pose_msg.header.frame_id = "world";
      target_pose_msg.pose.position.x = 0.6;
      target_pose_msg.pose.position.y = -0.2;
      target_pose_msg.pose.position.z = 0.0 + g_object_depth/2;
      target_pose_msg.pose.orientation.w = 1.0;



      // tf2::Quaternion q;
      // q.setRPY(0, 0, -M_PI/2);   // roll=0, pitch=0, yaw=+90°
      // target_pose_msg.pose.orientation.x = q.x();
      // target_pose_msg.pose.orientation.y = q.y();
      // target_pose_msg.pose.orientation.z = q.z();
      // target_pose_msg.pose.orientation.w = q.w();



      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

      // Compute IK
      // clang-format off
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("ComputeIK", std::move(stage));
      // clang-format on
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame("object");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      wrapper->properties().set("comment", "(배치점) 소재 배치점 역기구학 계산");
      place->insert(std::move(wrapper));
    }

    {
      // open hand
      auto stage = std::make_unique<mtc::stages::MoveTo>("Place Object", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("gripper_open");
      stage->properties().set("comment", "(배치점) 그리퍼 핑거를 release하여 소재를 배치할때까지 대기");
      place->insert(std::move(stage));
    }

    {
      // clang-format off
      // forbid collision
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("Forbid Collision");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             false);
      stage->properties().set("comment", "(배치점) 그리퍼 핑거를 release하여 소재를 배치할때까지 대기");
      // clang-format on
      place->insert(std::move(stage));
    }

    {
      // detach object
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("Detach Object");
      stage->detachObject("object", hand_frame);
      stage->properties().set("comment", "(배치점) 그리퍼 핑거를 release하여 소재를 배치할때까지 대기");
      place->insert(std::move(stage));
    }

    {
      // backward_retreat
      auto stage = std::make_unique<mtc::stages::MoveRelative>("Retreat from place", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.12, 0.35);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");
      stage->properties().set("comment", "(배치 후퇴점) 소재 배치 후 후퇴를 위해 직선 상승 이동 ");

      // Set retreat direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }
    task.add(std::move(place));
  }

  {
    // Home Pos //
    auto stage_home_pos =
      std::make_unique<mtc::stages::MoveTo>("Return to Home", interpolation_planner);
    // 객체 속성 설정
    stage_home_pos->setGroup(arm_group_name);
    stage_home_pos->setGoal("home");
    stage_home_pos->properties().set("comment", "(배치 복귀 자세) 소재를 배치 후 복귀 자세. joint_space_move.");
    // task add
    task.add(std::move(stage_home_pos));
  }


  return task;
}






    /****************************************************
  ---- *                 main Program               *
     ***************************************************/
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  wp = BOX;


  mtc_task_node->setupPlanningScene(wp);
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
