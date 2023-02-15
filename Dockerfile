FROM tiryoh/ros2-desktop-vnc:galactic
ENV GAZEBO_MODEL_PATH /home/ubuntu/Desktop/gazebo_models
WORKDIR /home/ubuntu/Desktop
RUN git clone https://github.com/osrf/gazebo_models.git
WORKDIR /home/ubuntu/Desktop
RUN apt-get update && apt-get install -y \
    ros-galactic-turtlebot3* \
    ros-galactic-navigation* \
    && rm -rf /var/lib/apt/lists/*
ENV TURTLEBOT3_MODEL burger
COPY asound.conf /etc/asound.conf
ENV LIBGL_ALWAYS_SOFTWARE 1
RUN mkdir -p /home/ubuntu/Desktop/colcon_ws/src
WORKDIR /home/ubuntu/Desktop/colcon_ws/src
COPY workspace.repos /home/ubuntu/Desktop/colcon_ws/src/workspace.repos