cmake_minimum_required(VERSION 3.8)
project(vesc_nexus)

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
find_package(Threads)

###########
## Build ##
###########

# node library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/can_interface.cpp
  src/message_translator.cpp
  src/odometry_publisher.cpp
  src/vesc_can_driver_node.cpp
  src/vesc_handler.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${CMAKE_THREAD_LIBS_INIT}
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN VescCanDriverNode
  EXECUTABLE vesc_can_driver_node
)

# Новая shared-библиотека для hardware_interface
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
add_library(vesc_hardware_interface SHARED
  src/vesc_system_hardware_interface.cpp
)
target_link_libraries(vesc_hardware_interface
  ${PROJECT_NAME}
  hardware_interface
)
target_include_directories(vesc_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(vesc_hardware_interface
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

# Экспорт pluginlib
pluginlib_export_plugin_description_file(hardware_interface vesc_nexus_plugins.xml)

# Установка
install(TARGETS vesc_hardware_interface
  DESTINATION lib
)


#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)