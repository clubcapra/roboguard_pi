cmake_minimum_required(VERSION 3.8)
project(roboguard_actions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(capra_control_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(actions_node src/roboguard_actions.cpp)
ament_target_dependencies(actions_node rclcpp trajectory_msgs control_msgs sensor_msgs std_msgs capra_control_msgs nav_msgs)

install(TARGETS
  actions_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
