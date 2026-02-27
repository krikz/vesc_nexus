cmake_minimum_required(VERSION 3.8)
project(vesc_nexus)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ═══════════════════════════════════════════════════════════════
# Dependencies
# ═══════════════════════════════════════════════════════════════
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(vesc_msgs REQUIRED)
find_package(Threads REQUIRED)

# ═══════════════════════════════════════════════════════════════
# Hardware Interface Library (ros2_control plugin)
# ═══════════════════════════════════════════════════════════════
add_library(vesc_hardware_interface SHARED
  src/vesc_system_hardware_interface.cpp
  src/vesc_handler.cpp
  src/can_interface.cpp
  src/message_translator.cpp
)

target_include_directories(vesc_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(vesc_hardware_interface
  ${CMAKE_THREAD_LIBS_INIT}
)

ament_target_dependencies(vesc_hardware_interface
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  vesc_msgs
)

# ═══════════════════════════════════════════════════════════════
# Install & Export
# ═══════════════════════════════════════════════════════════════
pluginlib_export_plugin_description_file(hardware_interface vesc_nexus_plugins.xml)

install(TARGETS vesc_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Install calibration tools
install(DIRECTORY tools/
  DESTINATION share/${PROJECT_NAME}/tools
  PATTERN "*.py"
  PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
)

# ═══════════════════════════════════════════════════════════════
# Testing
# ═══════════════════════════════════════════════════════════════
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(vesc_hardware_interface)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  vesc_msgs
)

ament_package()