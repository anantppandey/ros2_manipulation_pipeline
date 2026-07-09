cmake_minimum_required(VERSION 3.10)
project(giraffe_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)  
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

add_executable(moveit_trial src/moveit_trial.cpp)
ament_target_dependencies(moveit_trial
  rclcpp
  moveit_ros_planning_interface
)

add_executable(red_cube_detector src/red_cube_detector.cpp)
ament_target_dependencies(red_cube_detector
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  image_transport
  message_filters
  tf2_ros
  tf2_geometry_msgs
  tf2
  geometry_msgs
)

add_executable(cube_picker src/cube_picker.cpp)
ament_target_dependencies(cube_picker
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

add_executable(end_effector_pose src/end_effector_pose.cpp)
ament_target_dependencies(end_effector_pose
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(
  DIRECTORY launch meshes models urdf rviz config src worlds
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS moveit_trial red_cube_detector cube_picker end_effector_pose
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()