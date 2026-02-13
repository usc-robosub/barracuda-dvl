cmake_minimum_required(VERSION 3.5)
project(waterlinked_dvl)

find_package(ament_cmake REQUIRED)

# Install Python script
install(PROGRAMS
  scripts/barracuda-dvl-ros-driver.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

ament_package()