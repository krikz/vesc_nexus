cmake_minimum_required(VERSION 3.8)
project(vesc_nexus)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Найти зависимости
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Найти или объявить сообщения
# Генерация сообщений
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VescState.msg"
  "msg/VescStateStamped.msg"
  DEPENDENCIES builtin_interfaces
)

# Исходники
set(SOURCES
  src/vesc_nexus_node.cpp
  src/can_interface.cpp
  src/vesc_manager.cpp
  src/vesc_device.cpp
  src/packet_handler.cpp
)

set(INCLUDES
  include/vesc_nexus/can_interface.hpp
  include/vesc_nexus/vesc_manager.hpp
  include/vesc_nexus/vesc_device.hpp
  include/vesc_nexus/packet_handler.hpp
  include/vesc_nexus/vesc_nexus_node.hpp
)

add_library(vesc_nexus_core SHARED ${SOURCES})
target_include_directories(vesc_nexus_core PUBLIC include)
ament_target_dependencies(vesc_nexus_core
  "rclcpp"
  "std_msgs"
  "diagnostic_msgs"
  "geometry_msgs"
)

add_executable(vesc_nexus_node src/vesc_nexus_node.cpp)
ament_target_dependencies(vesc_nexus_node
  "rclcpp"
  "std_msgs"
  "diagnostic_msgs"
  "geometry_msgs"
)
target_link_libraries(vesc_nexus_node vesc_nexus_core)

# Установка
install(TARGETS
  vesc_nexus_core
  vesc_nexus_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES
  launch/vesc_nexus_node.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES
  config/vesc_nexus_example.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()