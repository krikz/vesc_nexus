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

# Генерация сообщений
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VescState.msg"
  "msg/VescStateStamped.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

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

# Зависимости для библиотеки — после add_library
add_dependencies(vesc_nexus_core ${PROJECT_NAME}_rosidl_typesupport_c)
add_dependencies(vesc_nexus_core ${PROJECT_NAME}_rosidl_typesupport_cpp)

# Включаемые директории
target_include_directories(vesc_nexus_core PUBLIC include)

# Зависимости от ROS 2
ament_target_dependencies(vesc_nexus_core
  "rclcpp"
  "std_msgs"
  "diagnostic_msgs"
  "geometry_msgs"
)

# Линковка с типами поддержки сообщений
target_link_libraries(vesc_nexus_core
  ${PROJECT_NAME}_rosidl_typesupport_c
  ${PROJECT_NAME}_rosidl_typesupport_cpp
)

# Создаём исполняемый файл (обязательно после add_library и до add_dependencies для него)
add_executable(vesc_nexus_node src/vesc_nexus_node.cpp)

# Зависимости для исполняемого файла — ТОЛЬКО ПОСЛЕ add_executable
add_dependencies(vesc_nexus_node ${PROJECT_NAME}_rosidl_typesupport_c)
add_dependencies(vesc_nexus_node ${PROJECT_NAME}_rosidl_typesupport_cpp)

# Зависимости для узла
ament_target_dependencies(vesc_nexus_node
  "rclcpp"
  "std_msgs"
  "diagnostic_msgs"
  "geometry_msgs"
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