cmake_minimum_required(VERSION 3.8)
project(video_processor)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(include)

# Create the video processor node executable
add_executable(video_processor_node
  src/video_processor_node.cpp
)

ament_target_dependencies(video_processor_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  auto_aim_interfaces
  OpenCV
)

# Install targets
install(TARGETS video_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
