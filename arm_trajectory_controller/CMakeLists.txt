cmake_minimum_required(VERSION 3.8)
project(arm_trajectory_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ament and ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Add the node executable
add_executable(arm_trajectory_controller_node
  src/arm_trajectory_controller_node.cpp
  src/main.cpp
)


# Include directories for headers
target_include_directories(arm_trajectory_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Require C99 and C++17
target_compile_features(arm_trajectory_controller_node PUBLIC
  c_std_99
  cxx_std_17
)

# Link in ROS 2 libraries (includes both rclcpp and trajectory_msgs)
ament_target_dependencies(arm_trajectory_controller_node
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
  trajectory_msgs
)

# Install the node
install(TARGETS arm_trajectory_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Testing / linting (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
