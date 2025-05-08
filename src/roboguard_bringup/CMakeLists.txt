cmake_minimum_required(VERSION 3.8)
project(roboguard_bringup)

# find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
