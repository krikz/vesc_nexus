cmake_minimum_required(VERSION 3.8)
project({package_name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Source files
set(SOURCES
  src/vesc_nexus_node.cpp
  src/can_interface.cpp
  src/vesc_manager.cpp
  src/vesc_device.cpp
  src/packet_handler.cpp
)

set(INCLUDES
  include/{package_name}/can_interface.hpp
  include/{package_name}/vesc_manager.hpp
  include/{package_name}/vesc_device.hpp
  include/{package_name}/packet_handler.hpp
  include/{package_name}/vesc_nexus_node.hpp
)

add_library({package_name}_core SHARED ${{SOURCES}})
target_include_directories({package_name}_core PUBLIC include)
ament_target_dependencies({package_name}_core
  "rclcpp"
  "std_msgs"
  "diagnostic_msgs"
  "geometry_msgs"
)

add_executable(${package_name}_node src/vesc_nexus_node.cpp)
ament_target_dependencies(${package_name}_node
  "rclcpp"
  "std_msgs"
  "diagnostic_msgs"
  "geometry_msgs"
)
target_link_libraries(${package_name}_node {package_name}_core)

install(TARGETS
  {package_name}_core
  ${package_name}_node
  DESTINATION lib/${{PROJECT_NAME}}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES
  launch/vesc_nexus_node.launch.py
  DESTINATION share/${{PROJECT_NAME}}/launch
)

install(FILES
  config/vesc_nexus_example.yaml
  DESTINATION share/${{PROJECT_NAME}}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VescState.msg"
  "msg/VescStateStamped.msg"
)