cmake_minimum_required(VERSION 3.8)
project(gazebo_contact_ros2_control)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros2_control REQUIRED)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wconversion -Wno-sign-conversion -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

include_directories(include)
link_directories(
  ${gazebo_dev_LIBRARY_DIRS}
)

# Libraries
# add_library(${PROJECT_NAME} SHARED
#   src/gazebo_cotnact_ros2_control_plugin.cpp
# )
# ament_target_dependencies(${PROJECT_NAME}
#   angles
#   controller_manager
#   gazebo_dev
#   gazebo_ros
#   hardware_interface
#   pluginlib
#   rclcpp
#   yaml_cpp_vendor
#   gazebo_ros2_control
# )


SET(HARDWARE_PLUGIN_NAME gazebo_contact_hardware_plugins)
add_library(${HARDWARE_PLUGIN_NAME} SHARED
   src/gazebo_contact_system.cpp
)
ament_target_dependencies(${HARDWARE_PLUGIN_NAME}
   angles
   gazebo_dev
   hardware_interface
   rclcpp
   gazebo_ros2_control
)

## Install
install(TARGETS 
  #${PROJECT_NAME}
  ${HARDWARE_PLUGIN_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  include
)
ament_export_dependencies(
  ament_cmake
  angles
  controller_manager
  gazebo_dev
  gazebo_ros
  hardware_interface
  pluginlib
  rclcpp
  yaml_cpp_vendor
  gazebo_ros2_control
)

ament_export_libraries(
  #${PROJECT_NAME}
  ${HARDWARE_PLUGIN_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(${PROJECT_NAME} ${HARDWARE_PLUGIN_NAME}.xml)

ament_package()
