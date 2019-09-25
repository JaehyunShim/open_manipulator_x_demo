################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(open_manipulator_x_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(open_manipulator_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)

set(dependencies
  "open_manipulator_msgs"
  "rclcpp"
  "sensor_msgs"
)

set(NODE_NAME "open_manipulator_x_demo")

add_executable(${NODE_NAME} src/open_manipulator_x_demo.cpp)
ament_target_dependencies(${NODE_NAME} ${dependencies})

################################################################################
# Install
################################################################################
install(TARGETS ${NODE_NAME}
  DESTINATION lib/${PROJECT_NAME} 
)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(open_manipulator_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_package()
