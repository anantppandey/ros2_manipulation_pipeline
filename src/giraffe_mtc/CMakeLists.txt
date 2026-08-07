cmake_minimum_required(VERSION 3.8)
project(giraffe_mtc)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(vision_msgs REQUIRED)

find_package(moveit_task_constructor_core REQUIRED)
find_package(giraffe_gazebo_plugins REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PickPlace.action"
  DEPENDENCIES geometry_msgs
)

add_executable(mtc_node
    src/mtc_node.cpp
)

ament_target_dependencies(mtc_node
  rclcpp
  moveit_ros_planning_interface
  moveit_task_constructor_core
  rclcpp_action
  giraffe_gazebo_plugins
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mtc_node "${cpp_typesupport_target}")

target_link_libraries(mtc_node
    moveit_task_constructor_core::moveit_task_constructor_core
    moveit_task_constructor_core::moveit_task_constructor_core_stages
)

add_executable(pick_place_orchestrator src/pick_place_orchestrator.cpp)
ament_target_dependencies(pick_place_orchestrator rclcpp rclcpp_action vision_msgs)
target_link_libraries(pick_place_orchestrator "${cpp_typesupport_target}")
install(TARGETS pick_place_orchestrator DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch src 
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
    mtc_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()