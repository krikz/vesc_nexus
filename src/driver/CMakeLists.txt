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

# Найти зависимости
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)

# Исходники
set(SOURCES
  src/vesc_nexus_node.cpp
  src/can_interface.cpp
  src/vesc_manager.cpp
  src/vesc_device.cpp
  src/packet_handler.cpp
)

# Создаём библиотеку
add_library(vesc_nexus_core SHARED ${SOURCES})
target_include_directories(vesc_nexus_core PUBLIC include)
ament_target_dependencies(vesc_nexus_core
  "rclcpp"
  "std_msgs"
  "diagnostic_msgs"
  "geometry_msgs"
)

# Создаём исполняемый файл
add_executable(vesc_nexus_node src/vesc_nexus_node.cpp)
target_include_directories(vesc_nexus_node PUBLIC include)
ament_target_dependencies(vesc_nexus_node
  "rclcpp"
  "std_msgs"
  "diagnostic_msgs"
  "geometry_msgs"
)

# Линковка библиотеки
target_link_libraries(vesc_nexus_core
  vesc_msgs__rosidl_typesupport_c
  vesc_msgs__rosidl_typesupport_cpp
)

# Тесты
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED) 
  ament_lint_auto_find_test_dependencies()
endif()

# Обязательно в конце
ament_package()