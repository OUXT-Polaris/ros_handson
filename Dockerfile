FROM tiryoh/ros2-desktop-vnc:humble
RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot3* \
    && rm -rf /var/lib/apt/lists/*
ENV TURTLEBOT3_MODEL burger
RUN mkdir -p /home/colcon_ws
WORKDIR /home/colcon_ws
COPY asound.conf /etc/asound.conf
ENV LIBGL_ALWAYS_SOFTWARE 1