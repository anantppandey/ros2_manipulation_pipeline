cmake_minimum_required(VERSION 3.8)
project(giraffe_mtc)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

find_package(moveit_task_constructor_core REQUIRED)

add_executable(mtc_node
    src/mtc_node.cpp
)

ament_target_dependencies(
    mtc_node
    rclcpp
    moveit_core
    moveit_ros_planning_interface
)

target_link_libraries(mtc_node
    moveit_task_constructor_core::moveit_task_constructor_core
    moveit_task_constructor_core::moveit_task_constructor_core_stages
)

install(TARGETS
    mtc_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()