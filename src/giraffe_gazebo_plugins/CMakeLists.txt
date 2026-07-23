cmake_minimum_required(VERSION 3.16)
project(giraffe_gazebo_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# ---- ROS 2 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Threads REQUIRED)

# ---- Gazebo Harmonic (gz-sim8) ----
# gz-cmake3 gives us the gz_find_package() helper used below; this is the
# same pattern ros-controls/gz_ros2_control and other ROS-2-on-Harmonic
# packages use to mix ament_cmake with the Gazebo library set.
find_package(gz-cmake3 REQUIRED)

gz_find_package(gz-common5 REQUIRED)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

add_library(giraffe_gazebo_plugins SHARED
  src/gripper_attach_plugin.cpp
)

target_include_directories(giraffe_gazebo_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(giraffe_gazebo_plugins
  rclcpp
  std_srvs
)

target_link_libraries(giraffe_gazebo_plugins
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  Threads::Threads
)

# NOTE: gz-sim looks for plugin .so files on GZ_SIM_SYSTEM_PLUGIN_PATH.
# Installing to lib/ (not lib/${PROJECT_NAME}/) matches the bare filename
# already used in the xacro (`libgiraffe_gazebo_plugins.so`) and matches
# what the launch file change below points GZ_SIM_SYSTEM_PLUGIN_PATH at.
install(
  TARGETS giraffe_gazebo_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_package()
