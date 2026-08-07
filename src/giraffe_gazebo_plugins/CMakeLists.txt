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
find_package(rosidl_default_generators REQUIRED)
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

# ---- Custom service, generated in THIS package ----
# rosidl_generate_interfaces() creates a CMake target literally named
# ${PROJECT_NAME} (giraffe_gazebo_plugins) to drive codegen. That's why the
# C++ library target below is named giraffe_gazebo_plugins_lib instead of
# giraffe_gazebo_plugins -- two CMake targets can't share a name. The
# installed .so file is still renamed back to libgiraffe_gazebo_plugins.so
# via OUTPUT_NAME, so the xacro plugin declaration doesn't change at all.
# This is the standard ROS 2 "define and use an interface in the same
# package" pattern -- see:
# https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AttachDetach.srv"
)
ament_export_dependencies(rosidl_default_runtime)

add_library(giraffe_gazebo_plugins_lib SHARED
  src/gripper_attach_plugin.cpp
)
set_target_properties(giraffe_gazebo_plugins_lib PROPERTIES
  OUTPUT_NAME giraffe_gazebo_plugins
)

target_include_directories(giraffe_gazebo_plugins_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(giraffe_gazebo_plugins_lib
  rclcpp
)

# Link the plugin library against the AttachDetach service types generated
# above, in this same package.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(giraffe_gazebo_plugins_lib
  "${cpp_typesupport_target}"
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  Threads::Threads
)

# NOTE: gz-sim looks for plugin .so files on GZ_SIM_SYSTEM_PLUGIN_PATH.
# Installing to lib/ (not lib/${PROJECT_NAME}/) matches the bare filename
# already used in the xacro (`libgiraffe_gazebo_plugins.so`).
install(
  TARGETS giraffe_gazebo_plugins_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_package()
