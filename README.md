# Setup

Just run
```
sh setup.sh
```

# Run docker image
## Nviia GPUがある場合
```
rocker --nvidia --x11 ros_handson rviz2
```

## Nviia GPUがない場合
```
rocker --x11 ros_handson ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```