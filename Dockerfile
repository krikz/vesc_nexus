# Используем базовый образ ROS 2 Humble (подходит для Raspberry Pi)
FROM introlab3it/rtabmap_ros:humble-latest

# Установка системных зависимостей
RUN apt-get update && apt-get install -y \
    build-essential \
    libffi-dev \
    python3-dev \
    python3-pip \
    can-utils \
    iproute2 \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

# Рабочая директория
WORKDIR /ws
RUN mkdir -p src

# Копируем только ваш пакет vesc_nexus
COPY . src/vesc_nexus

# Сборка пакета
RUN . /opt/ros/humble/setup.sh && \
    colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# Добавляем setup.bash в окружение
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc

# Настройка ROS 2
ENV ROS_DOMAIN_ID=0
ENV ROS_LOCALHOST_ONLY=0

# Команда по умолчанию — запуск драйвера
CMD ["bash", "-c", "source /ws/install/setup.bash && ros2 launch vesc_driver vesc_driver_node.launch.py"]