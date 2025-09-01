# CMakeLists.txt для сообщений
cmake_minimum_required(VERSION 3.8)
project(vesc_nexus_msgs)

# Найти rosidl_generator_c для генерации сообщений
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Определите сообщения
set(MSG_FILES
  "VescState.msg"
  "VescStateStamped.msg"
)

# Генерация интерфейсов
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES builtin_interfaces
)