FROM ros:humble
RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot3* \
    && rm -rf /var/lib/apt/lists/*
ENV TURTLEBOT3_MODEL burger
RUN mkdir -p /home/colcon_ws
WORKDIR /home/colcon_ws