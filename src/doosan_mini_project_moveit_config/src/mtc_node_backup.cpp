// ROS2 및 Moveit2 라이브러리

// RCLCPP
#include <rclcpp/rclcpp.hpp>

// 로봇 모델 & 충돌 객체(Collision Objects)와 상호작용하는 기능
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

// Moveit Task Constructor의 구성요소
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

// TF2와 geometry_msgs 간 변환 지원
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

// TF2와 Eigen 간 좌표&자세 변환 지원
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif



// 로거 추가
static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
// 네임스페이스 생성
namespace mtc = moveit::task_constructor;




// Moveit Task Constructor 클래스
class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};


// 노드 초기화
MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}


// 노드 기본 인터페이스 getter
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}



// 플래닝 씬 셋업
void MTCTaskNode::setupPlanningScene()
{

    // 오브젝트 생성
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = -0.25;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}


// MoveIt Task Constructor의 Task 객체와 상호작용하는 함수
void MTCTaskNode::doTask()
{
  // 1. Task 객체 생성 (스테이지 추가, 속성 설정)
  task_ = createTask();

  try
  {
    // 2. Task 초기화 
    // ( Stage들이 올바르게 연결되어 있는지, 
    //   Planner(Solver)가 제대로 설정되었는지, 
    //   Property가 모두 전달되었는지, 
    //   Planning Scene을 생성, 
    //   각 Stage를 사용할 수 있도록 준비 )
    // --> Task를 실행하기 전에 내부 구조를 검증하고 준비하는 과정
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e); // InitStageException
    return;
  }




// 3. 경로 계획 : 성공적인 계획을 지정된 갯수만큼 찾을 때까지 탐색
//              --> 성공한 Solution의 개수
  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }

// 4. Rviz 시각화 :  Solution 토픽 Publish (Rviz 플러그인과 연결된 Action Server 인터페이스)
  task_.introspection().publishSolution(*task_.solutions().front());


// 5. 계획 실행 (Moveit의 Action server 인터페이스)
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}




// MoveIt Task Constructor(MTC)의 Task 객체를 생성하고 몇 가지 초기 속성(Property)을 설정
// 하나의 Task를 사용할 수 있도록 기본 설정을 모두 해놓는 것

mtc::Task MTCTaskNode::createTask()
{
    mtc::Task task;

    // Task의 이름을 설정 (RViz, Introspection, Debug 출력)
    task.stages()->setName("demo task");
    // 로봇 모델(Robot Model)을 로드 
    // (URDF + SRDF 읽음 -> Moveit RobotModel 객체 생성: Link, Joint, Joint Limits, Planning Group, End Effector, Collision 정보)
    task.loadRobotModel(node_);


    // 자주 사용할 몇 가지 좌표 프레임(Frame)의 이름을 정의 (문자열)
    const auto& arm_group_name = "arm";
    const auto& hand_group_name = "hand";
    const auto& hand_frame = "rh_p12_rn_base";

    // 프레임 이름들을 Task의 Property로 등록 (group, eef, ik_frame)
    // Task 전체에서 공유하는 설정 값 (모든 Stage가 해당 설정을 사용.)
    task.setProperty("group", arm_group_name);
    task.setProperty("eef", hand_group_name);
    task.setProperty("ik_frame", hand_frame);




    // Disable warnings for this line, as it's a variable that's set but not used in this example
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
    #pragma GCC diagnostic pop



    // CurrentState Stage 추가 (현재 상태 초기화)
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.01);



    // MoveTo Stage 추가 (그리퍼 열기)
    auto stage_open_hand =
        std::make_unique<mtc::stages::MoveTo>("gripper_open", interpolation_planner);
    stage_open_hand->setGroup(hand_group_name);
    stage_open_hand->setGoal("gripper_open");
    task.add(std::move(stage_open_hand));


    // 커넥터 스테이지
    auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
        "move to pick",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    stage_move_to_pick->setTimeout(5.0);
    stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_pick));

    mtc::Stage* attach_object_stage =
        nullptr;  // Forward attach_object_stage to place pose generator



    {
        auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
        task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
        grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                                { "eef", "group", "ik_frame" });

        {
            auto stage =
                std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", hand_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(0.1, 0.15);

            // Set hand forward direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }


        {
            // Sample grasp pose
            auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "grasp_pose");
            stage->setPreGraspPose("gripper_open");
            stage->setObject("object");
            stage->setAngleDelta(M_PI / 12);
            stage->setMonitoredStage(current_state_ptr);  // Hook into current state

            Eigen::Isometry3d grasp_frame_transform;
            Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                                Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
            grasp_frame_transform.linear() = q.matrix();
            grasp_frame_transform.translation().z() = 0.1;

            // Compute IK
            auto wrapper =
                std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(8);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame(grasp_frame_transform, hand_frame);
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
            grasp->insert(std::move(wrapper));
        }

        {
            auto stage =
                std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
            stage->allowCollisions("object",
                                    task.getRobotModel()
                                        ->getJointModelGroup(hand_group_name)
                                        ->getLinkModelNamesWithCollisionGeometry(),
                                    true);
            grasp->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("gripper_close", interpolation_planner);
            stage->setGroup(hand_group_name);
            stage->setGoal("gripper_close");
            grasp->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
            stage->attachObject("object", hand_frame);
            attach_object_stage = stage.get();
            grasp->insert(std::move(stage));
        }

        {
            auto stage =
                std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
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


        task.add(std::move(grasp));

    }




    
  return task;
}

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

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}