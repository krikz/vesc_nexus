cmake_minimum_required(VERSION 3.8)
project(vesc_driver)

# Set minimum C++ standard to C++14
if(NOT "${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++14")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif("${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}" STREQUAL "98")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++14")
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Threads REQUIRED)

# Sources
set(SOURCES
  src/vesc_nexus_node.cpp
  src/can_interface.cpp
  src/vesc_manager.cpp
  src/vesc_device.cpp
  src/packet_handler.cpp
)

# Library
ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  diagnostic_msgs
  geometry_msgs
  vesc_msgs
)
target_link_libraries(${PROJECT_NAME} Threads::Threads)

# Executable
ament_auto_add_executable(${PROJECT_NAME}_node src/vesc_nexus_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  diagnostic_msgs
  geometry_msgs
  vesc_msgs
)

# Install files
ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)