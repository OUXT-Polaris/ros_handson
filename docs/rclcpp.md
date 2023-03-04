# 自作ノードの作り方(C++)

## パッケージの作成

すべてのROS2ノードはパッケージに属している必要があります。`ros2 launch`や`ros2 run`コマンド等でROS2ノードを起動する際には必ずパッケージ名が必要になります。  
本教材で出来上がるコードは[こちらのリポジトリ](https://github.com/OUXT-Polaris/ros_handson_packages.git)に収録しています。参考にしてみてください。  
また、コードの詳細な解説はコードのコメントに残してあります。ぜひご参照ください。  
`ros2 pkg create`コマンドを使用し、実際にパッケージを作ってみましょう。  

```bash
cd /home/ubuntu/Desktop/colcon_ws/src
ros2 pkg create tutorial --dependencies rclcpp rclcpp_components ament_cmake_auto geometry_msgs --library-name publish --license Apache-2.0
```

このコマンドを実行すると`tutorial`という名前のROS2パッケージが出来上がります。

`--dependencies`オプションは依存関係を追加します。  
依存関係は`tutorial`パッケージは`rclcpp`パッケージに依存しています。
というふうに依存関係を`package.xml`に記述してあげることで`rclcpp`パッケージに含まれるリソースを`tutorial`パッケージで使えるようにしたりビルド順を制御したりします。  
`ament_cmake_auto`はament_cmakeを楽に記述させてくれるパッケージです。正直これがないと記述がとても冗長で面倒くさいので使用を推奨します。
[この記事](https://hans-robo.hatenablog.com/entry/2020/12/15/153503)が参考になります。

`--library-name`オプションは共有ライブラリを作るオプションです。
本教材では最短でROS2を使って良いロボットソフトウェアをかけるようになってもらうため、「コンポーネント志向」という書き方でノードを記述します。  
他の記述方法も存在するのですが、ROS2の利点であるシステム構成の柔軟性とノード間のレイテンシを最小化するににはこの書き方が最も優れています。  
というか現状rclcppで大掛かりなシステムを作ろうとするとこの方法しかありません。  

`--license`オプションはライセンスを設定します。
ライセンス？どうでもいいじゃんと思うかもしれませんが、ライセンスの書いていないソフトウェアは公開されていたとしても、使っていいかの判断が付きません。  
ちゃんと設定して他の人から使ってもらえるパッケージを作りましょう。  

## CMakeLists.txtの編集

パッケージを作った段階では、ament_cmake_autoは使われていません、このままではとても記述が面倒なのでCMakeLists.txtを[このファイル](https://github.com/OUXT-Polaris/ros_handson_packages/blob/master/tutorial/CMakeLists.txt)のように書き換え、ament_cmake_autoを使ってみましょう。
だいぶスッキリしたかと思います。  
では、次に共有ライブラリ側にPublisher/Subscriberを実装し、Pub/Sub通信をしていきたいと思います。  

## Publisher/Subscriberの実装
tutorialパッケージのincludeディレクトリに[subscribe.hpp](https://github.com/OUXT-Polaris/ros_handson_packages/blob/master/tutorial/include/tutorial/subscribe.hpp)と
[publish.hpp](https://github.com/OUXT-Polaris/ros_handson_packages/blob/master/tutorial/include/tutorial/publish.hpp)をコピーします。  
また、それとは別にマルチプラットフォーム対応マクロをまとめた[visibility_control.h](https://github.com/OUXT-Polaris/ros_handson_packages/blob/master/tutorial/include/tutorial/visibility_control.h)をコピーします。  
その後さらに[subscribe.cpp](https://github.com/OUXT-Polaris/ros_handson_packages/blob/master/tutorial/src/subscribe.cpp)および[publish.cpp](https://github.com/OUXT-Polaris/ros_handson_packages/blob/master/tutorial/src/publish.cpp)をtutorialパッケージのsrcディレクトリにコピーします。
最後に、[pub_sub_node.cpp](https://github.com/OUXT-Polaris/ros_handson_packages/blob/master/tutorial/src/pub_sub_node.cpp)をsrcディレクトリにコピーしましょう。

## コンパイル

全てのコードのコピーが終了したら、次はコンパイルです。  
コンパイルには`colcon build`コマンドを利用しますがその際にはワークスペースのルートディレクトリに移動する必要があります。  

```bash
cd /home/ubuntu/Desktop/colcon_ws
colcon build --symlink-install
```

colcon buildには様々なオプションがありますが、その中でも便利なものは[こちらの記事](https://qiita.com/seshimaru/items/ed344530ead80ab1733f)にまとめられています。

## 自作ノードの実行

コンパイルが完了したところで、上記の手順に従って作成したpub_sub_nodeという実行ファイルを実行してみましょう。

```bash
source install/local_setup.bash
ros2 run tutorial pub_sub_node
```

すると以下のような出力が得られるはずです。  

```bash
[INFO] [1677948977.302295115] [publish]: start initializing publisher
[INFO] [1677948977.305744085] [subscribe]: start initializing subscriber
[INFO] [1677948977.403264742] [publish]: Hello
[INFO] [1677948977.403477482] [subscribe]: Hello
[INFO] [1677948977.503363186] [publish]: Hello
[INFO] [1677948977.503643422] [subscribe]: Hello
[INFO] [1677948977.603355263] [publish]: Hello
[INFO] [1677948977.603684636] [subscribe]: Hello
[INFO] [1677948977.703275134] [publish]: Hello
[INFO] [1677948977.703552035] [subscribe]: Hello
[INFO] [1677948977.803279880] [publish]: Hello
[INFO] [1677948977.803548277] [subscribe]: Hello
[INFO] [1677948977.903299925] [publish]: Hello
[INFO] [1677948977.903680165] [subscribe]: Hello
[INFO] [1677948978.003363918] [publish]: Hello
[INFO] [1677948978.003732109] [subscribe]: Hello
[INFO] [1677948978.103375621] [publish]: Hello
[INFO] [1677948978.103689912] [subscribe]: Hello
[INFO] [1677948978.203297655] [publish]: Hello
[INFO] [1677948978.203575100] [subscribe]: Hello
[INFO] [1677948978.303349310] [publish]: Hello
[INFO] [1677948978.303621975] [subscribe]: Hello
[INFO] [1677948978.403307587] [publish]: Hello
[INFO] [1677948978.403619985] [subscribe]: Hello
[INFO] [1677948978.503364680] [publish]: Hello
[INFO] [1677948978.503618966] [subscribe]: Hello
```
