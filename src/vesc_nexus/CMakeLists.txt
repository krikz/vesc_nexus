cmake_minimum_required(VERSION 3.8)
project(vesc_nexus)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(vesc_msgs REQUIRED)
find_package(Threads REQUIRED)

# ros2_control hardware plugin
add_library(vesc_hardware_interface SHARED
  src/can_interface.cpp
  src/message_translator.cpp
  src/vesc_handler.cpp
  src/vesc_system_hardware_interface.cpp
)
target_include_directories(vesc_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(vesc_hardware_interface Threads::Threads)
ament_target_dependencies(vesc_hardware_interface
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  vesc_msgs
)

pluginlib_export_plugin_description_file(hardware_interface vesc_nexus_plugins.xml)

install(TARGETS vesc_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

# Calibration and manual-control tools (python-can based, run outside ROS)
install(DIRECTORY tools/
  DESTINATION share/${PROJECT_NAME}/tools
  USE_SOURCE_PERMISSIONS
  PATTERN "__pycache__" EXCLUDE
  PATTERN "tests" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_message_translator test/test_message_translator.cpp)
  target_link_libraries(test_message_translator vesc_hardware_interface)

  ament_add_gtest(test_vesc_handler test/test_vesc_handler.cpp)
  target_link_libraries(test_vesc_handler vesc_hardware_interface)
endif()

ament_export_include_directories(include)
ament_export_libraries(vesc_hardware_interface)
ament_export_dependencies(rclcpp rclcpp_lifecycle hardware_interface pluginlib vesc_msgs)

ament_package()
