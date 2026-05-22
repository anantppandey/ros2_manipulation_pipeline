cmake_minimum_required(VERSION 3.10)
project(giraffe_description)


find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(moveit_trial src/moveit_trial.cpp)

ament_target_dependencies(moveit_trial
  rclcpp
  moveit_ros_planning_interface
)

install(
  DIRECTORY launch meshes models urdf rviz config src worlds
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS moveit_trial
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
