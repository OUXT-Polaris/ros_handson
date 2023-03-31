# Braitenberg Vehicleを作ろう！

## Braitenberg Vehicleとは？
Braitenberg Vehicleはセンサをモータに直結して作られたロボットです。  
非常に単純な構造をしていますが、障害物回避といった実環境に適応した動きが実現できます。  

Turtlebot3のセンサはモーターに直結していませんが、センサ値から仮想的なモータ司令を生成し、そこから速度司令を計算することで擬似的にBraitenberg Vehicleを再現してみましょう。  

pythonでの実装例が[こちらの記事](https://qiita.com/domutoro406/items/c52892b5afd2de34d0fd)に載っています。  
今回のハンズオンでは、障害物を避けながらゴール地点へ向かう3号cを作っていきます。  

それでは、パッケージを作っていきましょう

```bash
cd /home/ubuntu/Desktop/colcon_ws/src/ros_handson_packages
ros2 pkg create braitenberg_vehicle --dependencies rclcpp rclcpp_components ament_cmake_auto geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs --library-name braitenberg_vehicle_controller --license Apache-2.0
```


