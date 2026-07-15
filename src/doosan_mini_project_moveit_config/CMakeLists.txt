cmake_minimum_required(VERSION 3.22)
project(doosan_mini_project_moveit_config)

# =========================

# Dependencies

# =========================

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)

# find_package(rosidl_default_generators REQUIRED)

find_package(banana_command REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(trajectory_msgs REQUIRED)

# =========================

# Custom messages

# =========================

# rosidl_generate_interfaces(${PROJECT_NAME}

# "msg/PickPlaceCommand.msg"

# DEPENDENCIES

# std_msgs

# geometry_msgs

# )

# =========================

# Executable: 기존 mtc_node

# =========================

add_executable(mtc_node
src/mtc_node.cpp
)

target_include_directories(mtc_node
PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)

target_compile_features(mtc_node
PUBLIC
cxx_std_17
)

ament_target_dependencies(mtc_node
rclcpp
ament_index_cpp
std_msgs
geometry_msgs
shape_msgs
moveit_msgs
moveit_ros_planning_interface
moveit_task_constructor_core
moveit_task_constructor_msgs
)

# =========================

# Executable: command subscriber + MTC task

# =========================

add_executable(pick_place_system
src/pick_place_command_node.cpp
src/mtc_task_node.cpp
)

target_include_directories(pick_place_system
PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)

target_compile_features(pick_place_system
PUBLIC
cxx_std_17
)

ament_target_dependencies(pick_place_system
rclcpp
banana_command
std_msgs
geometry_msgs
shape_msgs
moveit_msgs
moveit_ros_planning_interface
moveit_task_constructor_core
moveit_task_constructor_msgs
)

# =========================

# Executable: trajectory subscriber (json file saver)

# =========================

add_executable(
  mtc_solution_json_saver
  src/mtc_solution_json_saver.cpp
)

target_compile_features(
  mtc_solution_json_saver
  PUBLIC
  cxx_std_17
)

target_compile_definitions(
  mtc_solution_json_saver
  PRIVATE
  PACKAGE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

ament_target_dependencies(
  mtc_solution_json_saver
  rclcpp
  ament_index_cpp
  moveit_task_constructor_msgs
  moveit_msgs
  trajectory_msgs
  std_msgs
  builtin_interfaces
)

target_link_libraries(
  mtc_solution_json_saver
  nlohmann_json::nlohmann_json
  Threads::Threads
)
# =========================

# Link generated message

# =========================

# rosidl_get_typesupport_target(

# cpp_typesupport_target

# ${PROJECT_NAME}

# rosidl_typesupport_cpp

# )

# target_link_libraries(

# pick_place_system

# "${cpp_typesupport_target}"

# )

# add_dependencies(

# pick_place_system

# ${PROJECT_NAME}__rosidl_generator_cpp

# )

# =========================

# Install executables

# =========================

install(
TARGETS
mtc_node
pick_place_system
DESTINATION
lib/${PROJECT_NAME}
)

install(
  TARGETS mtc_solution_json_saver
  DESTINATION lib/${PROJECT_NAME}
)


# =========================

# Install headers

# =========================

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
install(
DIRECTORY include/
DESTINATION include
)
endif()

# =========================

# Install package resources

# =========================

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
install(
DIRECTORY launch
DESTINATION share/${PROJECT_NAME}
PATTERN "setup_assistant.launch" EXCLUDE
)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
install(
DIRECTORY config
DESTINATION share/${PROJECT_NAME}
)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.setup_assistant")
install(
FILES .setup_assistant
DESTINATION share/${PROJECT_NAME}
)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/trajectories")
install(
DIRECTORY trajectories
DESTINATION share/${PROJECT_NAME}
)
endif()

# =========================

# Export

# =========================

# 이 패키지에서 자체 ROS 인터페이스를 생성하지 않으므로

# rosidl_default_runtime export는 필요하지 않음.

#

# ament_export_dependencies(

# rosidl_default_runtime

# )

ament_package()
