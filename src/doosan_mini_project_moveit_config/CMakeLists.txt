cmake_minimum_required(VERSION 3.22)
project(doosan_mini_project_moveit_config)

# =========================

# Dependencies

# =========================

find_package(ament_cmake REQUIRED)
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
