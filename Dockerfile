FROM tiryoh/ros2-desktop-vnc:galactic
RUN apt-get update && apt-get install -y \
    ros-galactic-turtlebot3* \
    && rm -rf /var/lib/apt/lists/*
ENV TURTLEBOT3_MODEL burger
COPY asound.conf /etc/asound.conf
ENV LIBGL_ALWAYS_SOFTWARE 1
RUN mkdir -p /home/ubuntu/Desktop/colcon_ws/src
WORKDIR /home/ubuntu/Desktop/colcon_ws/src
COPY workspace.repos /home/ubuntu/Desktop/colcon_ws/src/workspace.repos