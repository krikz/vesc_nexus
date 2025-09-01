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
# Генерация сообщений — создаёт таргеты с ДВУМЯ подчёркиваниями: vesc_nexus__rosidl_*
target_include_directories(vesc_nexus_node PUBLIC include)
ament_target_dependencies(vesc_nexus_node
  "rclcpp"
  "std_msgs"
  "diagnostic_msgs"
  "geometry_msgs"
)



# Генерация сообщений — создаёт таргеты вида: vesc_nexus__rosidl_typesupport_*
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VescState.msg"
  "msg/VescStateStamped.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

# Теперь можно добавить зависимости от сгенерированных типов (только после rosidl_generate_interfaces!)
add_dependencies(vesc_nexus_core ${PROJECT_NAME}__rosidl_typesupport_c)
add_dependencies(vesc_nexus_core ${PROJECT_NAME}__rosidl_typesupport_cpp)

add_dependencies(vesc_nexus_node ${PROJECT_NAME}__rosidl_typesupport_c)
add_dependencies(vesc_nexus_node ${PROJECT_NAME}__rosidl_typesupport_cpp)

# Линковка библиотеки с поддержкой сообщений
target_link_libraries(vesc_nexus_core
  ${PROJECT_NAME}__rosidl_typesupport_c
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# Установка целей
install(TARGETS
  vesc_nexus_core
  vesc_nexus_node
  DESTINATION lib/${PROJECT_NAME}
)

# Установка заголовков
install(DIRECTORY include/
  DESTINATION include
)

# Установка launch-файлов
install(FILES
  launch/vesc_nexus_node.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

# Установка конфигов
install(FILES
  config/vesc_nexus_example.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

# Тесты (опционально)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Обязательно в конце
ament_package()

# Установка конфигурационных файлов
install(FILES
  config/vesc_nexus_example.yaml
  config/dual_can_config.yaml
  config/single_can_quad.yaml
  DESTINATION share/${PROJECT_NAME}/config
)