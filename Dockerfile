FROM ros:humble
RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot3* \
    && rm -rf /var/lib/apt/lists/*