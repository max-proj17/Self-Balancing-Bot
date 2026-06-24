cmake_minimum_required(VERSION 3.5)
project(balance_bot)

# Use C++17 by default
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Executable: Simple Subscriber Node
add_executable(balance_bot
        src/balance_controller.cpp
        src/kalman_filter.cpp
)

ament_target_dependencies(balance_bot rclcpp std_msgs sensor_msgs tf2 tf2_ros)

target_include_directories(balance_bot PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Install targets
install(TARGETS
  balance_bot
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF/Xacro files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY gazebo/config
  DESTINATION share/${PROJECT_NAME}/gazebo
)

install(
  DIRECTORY gazebo
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
