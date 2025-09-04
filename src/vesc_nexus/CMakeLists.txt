# Новая shared-библиотека для hardware_interface
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