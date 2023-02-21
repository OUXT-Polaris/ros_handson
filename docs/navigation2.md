# navigation2による自律移動

## navigation2とは？
navigation2は、ROS1時代に存在したnavigationパッケージの以下のような反省点を踏まえて再設計された自律移動のためのROS2パッケージ群です。  
- Lifecycleによる決定的launch
- 状態管理を有限状態機械からBehavior Treeに変更
- より詳細な部品化（navigationではlocal planner/global planner/localization/recoveryくらいしか主要な部品がない）

navigation2には[かなり詳細なドキュメント](https://navigation.ros.org/getting_started/index.html)が用意されており、自分のロボットに適用する際の大きな助けになるでしょう。

本教材では、navigation2を用いてgazebo上にspawnさせたturtlebot3をSLAMアルゴリズムで地図作成しながら自律移動させます。  
では、早速実際ハンズオン環境でnavigation2を使ってロボットを自律移動させていきましょう。

## navigation2によるTurtlebot3の自律移動

ターミナルを起動し、以下のコマンドを叩いてgazeboを立ち上げ、その世界にturtlebot3を召喚しましょう。

```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

次に、下記のコマンドでnavigation2を起動します。

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True slam:=True
```

`use_sim_time:=True`オプションはシミュレータやrosbagなどでロボットを動かすときに必要なオプションです。  
ROS2には以下の3種類の時間ソースがあります。

- system clock：ノードが走っているパソコンのシステム時刻
- ros clock：/clockトピックより取得された時刻、シミュレータでロボットを動かすときは時間を早めたり遅くしたりするのでこの時刻を使用
- steady clock：制御ループなどで使用する、巻き戻らない時刻

デフォルトで使われているのはsystem clockです。
シミュレータでros_clockを使用するには各ノードにuse_sim_timeパラメータをTrueに設定する必要があり、その設定を`ros2 launch`コマンド経由で引き渡せるのが`use_sim_time:=True`オプションです。
逆に実機でnavigation2を使用する場合には`use_sim_time:=Flase`を設定する必要があります。

`slam:=True`オプションはSLAMアルゴリズムを使用して地図を作りながら自己位置推定を行なうオプションです。  
これによって事前に地図を用意することなく自律移動を行なうことが可能です。
デフォルトで使用されれる設定ファイルをみてみると以下のようになっています。

<details>
<summary>/opt/ros/humble/share/slam_toolbox/config/mapper_params_online_sync.yaml</summary>

```bash
slam_toolbox:
  ros__parameters:

    # Plugin params
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    # ROS Parameters
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
    mode: mapping #localization

    # if you'd like to immediately start continuing a map at a given pose
    # or at the dock, but they are mutually exclusive, if pose is given
    # will use pose
    #map_file_name: test_steve
    #map_start_pose: [0.0, 0.0, 0.0]
    #map_start_at_dock: true

    debug_logging: false
    throttle_scans: 1
    transform_publish_period: 0.02 #if 0 never publishes odometry
    map_update_interval: 5.0
    resolution: 0.05
    max_laser_range: 20.0 #for rastering images
    minimum_time_interval: 0.5
    transform_timeout: 0.2
    tf_buffer_duration: 30.
    stack_size_to_use: 40000000 #// program needs a larger stack size to serialize large maps
    enable_interactive_mode: true

    # General Parameters
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.5
    minimum_travel_heading: 0.5
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1  
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
    do_loop_closing: true 
    loop_match_minimum_chain_size: 10           
    loop_match_maximum_variance_coarse: 3.0  
    loop_match_minimum_response_coarse: 0.35    
    loop_match_minimum_response_fine: 0.45

    # Correlation Parameters - Correlation Parameters
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1 

    # Correlation Parameters - Loop Closure Parameters
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    # Scan Matcher Parameters
    distance_variance_penalty: 0.5      
    angle_variance_penalty: 1.0    

    fine_search_angle_offset: 0.00349     
    coarse_search_angle_offset: 0.349   
    coarse_angle_resolution: 0.0349        
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
```
</details>

上記のyamlファイルを確認すると、Ceres SolverというGoogle社が開発しているオープンソース非線型最小二乗問題ソルバーを用いたloop closingありの2D SLAMを使用するというのがデフォルトの設定になっています。
もしこの設定を上書きしたい場合は、自分で上記のyamlファイルをコピーし保存したうえで[SLAM Toolboxの公式ドキュメント](https://github.com/SteveMacenski/slam_toolbox#configuration)を参照し必要なパラメータを書き換えた上で`ros2 launch`実行時の`slam_params_file`オプションにyamlファイルのパスを引き渡してください。

２つ目の`ros2 launch`コマンドを実行するとrviz2が立ち上がります。  
rviz2上で2D Goal Poseボタンをおして緑の矢印でゴール姿勢を指定するとロボットが自動的に指定地点に向かいます。  

<iframe width="1280" height="720" src="https://www.youtube.com/embed/-XdvmhwDhmo" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

## foxgloveによるrosbag可視化

navigation2を含むROS2ノードでは、一般的に`visualization_msgs/msg/MarkerArray`というrviz2にマーカー描画を行なうための専用型にマーカーの姿勢情報などを詰めてROS2 topicとしてpublishしています。  
この型を使ってデータをpublishしてそれをrosbagに記録しておけば[foxglove](https://foxglove.dev/)という極めて強力なツールを使用できます。  
[foxglove](https://foxglove.dev/)はもともと[webviz](https://github.com/cruise-automation/webviz)というプロジェクトのforkとしてスタートしました。  
[foxglove](https://foxglove.dev/)を用いるとrosbagをコマ送りしたりしてロボットの挙動を確認できるので非常に便利です。  
セットアップは下記のコマンドで完結します。  

```bash
sudo snap install foxglove-studio
```

ハンズオン環境にはすでに`foxglove-studio`をインストール済みですので、上記コマンドを実行する必要はありません。  

では、navigation2およびgazeboを立ち上げた状態で新たにシェルを立ち上げて以下のコマンドを実行しrosbagにデータを記録してください。  

```bash
ros2 bag record -a --storage mcap
```

rosbagを保存しながら、自律移動でturtlebot3をさまざまな場所に移動させてみてください。  
もういいかな、となったら`ctrl+c`でロギングを終了します。  

ロギングが完了したらrosbagを保存したディレクトリに移動し、以下のコマンドを実行します。

```bash
foxglove-studio
```

foxglove-studioが起動したら、Open Local Fileを選択し先ほど保存したrosbagを選択します。  
次に、3D Panelの右側にある歯車アイコンをクリックし、次に３つの点が並んでいるボタンを選びます。  
すると、Impoer/Export Settingsという文字があるのでそれをして下記のJsonをペーストします。

<details>
<summary>turtlebot3_navigation.json</summary>
```json
{
  "cameraState": {
    "perspective": true,
    "distance": 8.802533373035159,
    "phi": 58.95409645554882,
    "thetaOffset": 45,
    "targetOffset": [
      0,
      0,
      0
    ],
    "target": [
      0,
      0,
      0
    ],
    "targetOrientation": [
      0,
      0,
      0,
      1
    ],
    "fovy": 45,
    "near": 0.5,
    "far": 5000
  },
  "followMode": "follow-pose",
  "followTf": "map",
  "scene": {
    "transforms": {
      "enablePreloading": true,
      "editable": false,
      "showLabel": true,
      "axisScale": 0.5,
      "lineWidth": 0.5
    },
    "enableStats": false
  },
  "transforms": {
    "frame:": {
      "visible": false
    },
    "frame:base_link": {
      "visible": true
    },
    "frame:wheel_left_link": {
      "visible": false
    },
    "frame:wheel_right_link": {
      "visible": false
    },
    "frame:imu_link": {
      "visible": false
    },
    "frame:caster_back_link": {
      "visible": false
    },
    "frame:base_scan": {
      "visible": false
    },
    "frame:base_footprint": {
      "visible": false
    },
    "frame:odom": {
      "visible": false
    }
  },
  "topics": {
    "/global_costmap/costmap": {
      "visible": true
    },
    "/global_costmap/clearing_endpoints": {
      "visible": false,
      "colorField": "x",
      "colorMode": "colormap",
      "colorMap": "turbo"
    },
    "/cost_cloud": {
      "visible": true
    },
    "/slam_toolbox/scan_visualization": {
      "visible": true
    },
    "/robot_description": {
      "visible": false
    },
    "/transformed_global_plan": {
      "visible": false
    },
    "/marker": {
      "visible": false
    },
    "/scan": {
      "visible": false,
      "colorField": "intensity",
      "colorMode": "colormap",
      "colorMap": "turbo"
    },
    "/slam_toolbox/graph_visualization": {
      "visible": false
    },
    "/waypoints": {
      "visible": false
    },
    "/global_costmap/published_footprint": {
      "visible": false
    },
    "/plan": {
      "visible": false
    },
    "/local_plan": {
      "visible": false
    },
    "/local_costmap/published_footprint": {
      "visible": false
    }
  },
  "layers": {},
  "publish": {
    "type": "point",
    "poseTopic": "/move_base_simple/goal",
    "pointTopic": "/clicked_point",
    "poseEstimateTopic": "/initialpose",
    "poseEstimateXDeviation": 0.5,
    "poseEstimateYDeviation": 0.5,
    "poseEstimateThetaDeviation": 0.26179939
  }
}
```
</details>

すると、下記動画のようにrosbagのデータを可視化できました。

<iframe width="1280" height="720" src="https://www.youtube.com/embed/dbkO2k3_PrU" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

巻き戻しや早送りなどの操作が可能なので、これを使えばロボットのデバッグはかなり容易になります。  

<iframe width="1280" height="720" src="https://www.youtube.com/embed/M-lyLBgDjRM" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

foxgloveには無料では5GBまでと言った制限はあるものの、データ共有機能などもあるので「研究中に取れた生データを共有して先生に見てもらう」「部室で取れたデータを自宅に帰って分析する」などの使い方ができるので生産性をより高めることができます。  
