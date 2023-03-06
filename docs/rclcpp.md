# C++によるpub/sub通信

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

パッケージを作った段階では、ament_cmake_autoは使われていません、このままではとても記述が面倒なのでCMakeLists.txtを[このファイル](https://github.com/OUXT-Polaris/ros_handson_packages/blob/7f999bb0b8a75936acd005b375ddc2e40bc99640/tutorial/CMakeLists.txt)のように書き換え、ament_cmake_autoを使ってみましょう。
だいぶスッキリしたかと思います。  
では、次に共有ライブラリにPublisher/Subscriberを実装し、Pub/Sub通信をしていきたいと思います。  

## Publisher/Subscriberの実装
tutorialパッケージのincludeディレクトリに[subscribe.hpp](https://github.com/OUXT-Polaris/ros_handson_packages/blob/master/tutorial/include/tutorial/subscribe.hpp)と
[publish.hpp](https://github.com/OUXT-Polaris/ros_handson_packages/blob/master/tutorial/include/tutorial/publish.hpp)をコピーします。  
また、それとは別にマルチプラットフォーム対応マクロをまとめた[visibility_control.h](https://github.com/OUXT-Polaris/ros_handson_packages/blob/master/tutorial/include/tutorial/visibility_control.h)をコピーします。  
その後さらに[subscribe.cpp](https://github.com/OUXT-Polaris/ros_handson_packages/blob/master/tutorial/src/subscribe.cpp)および[publish.cpp](https://github.com/OUXT-Polaris/ros_handson_packages/blob/master/tutorial/src/publish.cpp)をtutorialパッケージのsrcディレクトリにコピーします。
最後に、[pub_sub_node.cpp](https://github.com/OUXT-Polaris/ros_handson_packages/blob/master/tutorial/src/pub_sub_node.cpp)をsrcディレクトリにコピーしましょう。

## コンパイル

すべてのコードのコピーが終了したら、次はコンパイルです。  
コンパイルには`colcon build`コマンドを利用しますがその際にはワークスペースのルートディレクトリに移動する必要があります。  

```bash
cd /home/ubuntu/Desktop/colcon_ws
colcon build --symlink-install
```

colcon buildにはさまざまなオプションがありますが、そのなかでも便利なものは[こちらの記事](https://qiita.com/seshimaru/items/ed344530ead80ab1733f)にまとめられています。

## 自作ノードの実行

コンパイルが完了したところで、上記の手順に従って作成したpub_sub_nodeという実行ファイルを実行してみましょう。

```bash
source install/local_setup.bash
ros2 run tutorial pub_sub_node
```

ros2 runコマンドはビルドした実行ファイルを1つ実行するコマンドです。  

```bash
ros2 run (パッケージ名) (実行ファイル名)
```

という形で命令を行ないます。

`ros2 run`コマンドを実行すると以下のような出力が得られるはずです。  

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

この結果はpublishノードから`Hello`という文字列が`/chatter`トピックに`std_msgs/msg/String`型のデータとして送り出され、それをsubscribeノードが受け取ってターミナルに出力しています。  
rqt_graphコマンドで確認してみましょう。  
新しいターミナルを立ち上げて以下のコマンドでノード構成を確認してみてください。

```bash
rqt_graph
```

すると下のような画像が表示されるかと思います。

![Not Found](images/rqt_graph_pub_sub_node.png)

ここで、ROS1時代からROSでシステムを作っている方には大きな違和感があるかと思います。  
ROS1時代は原則的に「1プロセス1ノード」でした、そのため1回`rosrun`コマンドを実行しただけでは２つのノードが起動することはありません。  
しかし、上の画像を見ると確かに2つのノードが1回`ros2 run`コマンドを実行するだけで起動しています。  
ここがROS1とROS2の大きな差です。  
ROS2は「1プロセス1ノード」の原則はありません、1つのプロセスに好きなだけノードを乗せることができます。(ただし載せすぎると思わぬ副作用を生じさせてしまったりするので何でもかんでも1つにしたほうがいいというわけではありません。)
同じプロセスに乗っているノードは計算機から見ると同じプロセスのプログラムなのでメモリアドレスの範囲を共有しており、ROS2はそれを利用して同じプロセスに乗っているノード間でのゼロコピー通信を行ないます。  

このゼロコピー通信は非常に高速で画像を圧縮する時間よりも生画像をゼロコピー通信で送るほうが高速であるという結果が得られている程です。  

<blockquote class="embedly-card"><h4><a href="https://qiita.com/Ke_N_551/items/d8637ddc806f94260ba8">ROS2で同一デバイス内画像通信の遅延について知りたくて色々試した話 - Qiita</a></h4><p>単一デバイス（Ultra96）内でROS2通信を利用して画像を送受信した場合、 画像のサイズ、圧縮するか否か、使用するDDS、などを変えて画像の送受信にかかる時間を測定・評価しました。どちらかというと通信遅延そのものについての評価というより、画像を送信する際にかかる時間の評価です。ですので、圧縮画像送信の際には画像の圧縮にかかる時間も遅延時間に含んでいたりします。</p></blockquote>
<script async src="//cdn.embedly.com/widgets/platform.js" charset="UTF-8"></script>

ROS2で安定したシステムを構築するには「どのノードとどのノードが同じプロセスに乗っているか？」を意識してコーディングおよびシステム設計を行なうことが重要です。  
筆者は普段の開発では以下の項目に留意しながら開発を進めています。  

- 頻繁に大容量の通信をするノード同士は同じプロセスに乗せる
    - 特にLiDAR/Cameraのデータは重く、レートもそこそこ高い
    - 認識後のデータは軽いので、センシング処理から認識処理までを1つのプロセスに固めることが多い
- トピックのドロップ率を下げたいノード同士は同じプロセスに乗せる
    - QoSによる再送機能はあるものの、パケット通信は確実に到達するわけではない

## launchファイルによる複数ノードの同時実行

ロボットシステムは非常に複雑で、1つの実行ファイルでロボットを動かそうとするのは困難です。  
そこで、ROS2には`ros2 launch`コマンドというコマンドがあります。  

前章で作った２つのノードを`ros2 launch`コマンドを使ってまとめて実行しましょう。

それでは、CMakeLists.txtを[このファイル](https://github.com/OUXT-Polaris/ros_handson_packages/blob/master/tutorial/CMakeLists.txt)のように書き換えてみましょう。
installという行が増えていると思いますが、これはROS2ではlaunchファイルや設定ファイルを使えるようにするにはinstallという作業をする必要があるからです。
実行ファイル等も本来はinstallする必要があるのですが、これはament_cmake_autoがやってくれています。

これが完了したら、もう一度colcon buildコマンドでビルドを行ないましょう。

```bash
cd /home/ubuntu/Desktop/colcon_ws
colcon build --symlink-install
```

colcon buildコマンドでビルドが完了したらいよいよ実行です。
下記のコマンドを実行してシステムを立ち上げてみてください。

```bash
ros2 launch tutorial pub_sub.launch.xml
```

すると以下のような出力が得られるはずです。

```bash
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [component_container_mt-1]: process started with pid [3596261]
[component_container_mt-1] [INFO] [1678104787.263613522] [pub_sub.pub_sub_container]: Load Library: /home/masaya-desktop/workspace/ros_handson_ws/install/tutorial/lib/libpublish.so
[component_container_mt-1] [INFO] [1678104787.265101658] [pub_sub.pub_sub_container]: Found class: rclcpp_components::NodeFactoryTemplate<tutorial::Publish>
[component_container_mt-1] [INFO] [1678104787.265137977] [pub_sub.pub_sub_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<tutorial::Publish>
[component_container_mt-1] [INFO] [1678104787.270249971] [pub_node]: start initializing publisher
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/pub_node' in container '/pub_sub/pub_sub_container'
[component_container_mt-1] [INFO] [1678104787.271713751] [pub_sub.pub_sub_container]: Load Library: /home/masaya-desktop/workspace/ros_handson_ws/install/tutorial/lib/libsubscribe.so
[component_container_mt-1] [INFO] [1678104787.272387091] [pub_sub.pub_sub_container]: Found class: rclcpp_components::NodeFactoryTemplate<tutorial::Subscribe>
[component_container_mt-1] [INFO] [1678104787.272424352] [pub_sub.pub_sub_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<tutorial::Subscribe>
[component_container_mt-1] [INFO] [1678104787.278872627] [sub_node]: start initializing subscriber
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/sub_node' in container '/pub_sub/pub_sub_container'
[component_container_mt-1] [INFO] [1678104787.371028786] [pub_node]: Hello
[component_container_mt-1] [INFO] [1678104787.371617488] [sub_node]: Hello
[component_container_mt-1] [INFO] [1678104787.471088200] [pub_node]: Hello
[component_container_mt-1] [INFO] [1678104787.471364157] [sub_node]: Hello
[component_container_mt-1] [INFO] [1678104787.571035671] [pub_node]: Hello
[component_container_mt-1] [INFO] [1678104787.571447180] [sub_node]: Hello
[component_container_mt-1] [INFO] [1678104787.671084154] [pub_node]: Hello
[component_container_mt-1] [INFO] [1678104787.671506773] [sub_node]: Hello
[component_container_mt-1] [INFO] [1678104787.770995355] [pub_node]: Hello
[component_container_mt-1] [INFO] [1678104787.771316282] [sub_node]: Hello
[component_container_mt-1] [INFO] [1678104787.871038601] [pub_node]: Hello
[component_container_mt-1] [INFO] [1678104787.871460026] [sub_node]: Hello
[component_container_mt-1] [INFO] [1678104787.971033180] [pub_node]: Hello
[component_container_mt-1] [INFO] [1678104787.971436789] [sub_node]: Hello
[component_container_mt-1] [INFO] [1678104788.071025164] [pub_node]: Hello
[component_container_mt-1] [INFO] [1678104788.071361172] [sub_node]: Hello
[component_container_mt-1] [INFO] [1678104788.171029467] [pub_node]: Hello
[component_container_mt-1] [INFO] [1678104788.171445897] [sub_node]: Hello
[component_container_mt-1] [INFO] [1678104788.271071987] [pub_node]: Hello
[component_container_mt-1] [INFO] [1678104788.271406651] [sub_node]: Hello
[component_container_mt-1] [INFO] [1678104788.371035315] [pub_node]: Hello
[component_container_mt-1] [INFO] [1678104788.371356692] [sub_node]: Hello
[component_container_mt-1] [INFO] [1678104788.471047626] [pub_node]: Hello
[component_container_mt-1] [INFO] [1678104788.471405888] [sub_node]: Hello
[component_container_mt-1] [INFO] [1678104788.571115586] [pub_node]: Hello
[component_container_mt-1] [INFO] [1678104788.571450181] [sub_node]: Hello
```

`ros2 run`コマンドで実行したときと同じような出力が得られていますね。  
コンパイル時にExecutorに乗せるか実行時にExecutorに乗せるかの差になってくるので例外処理などを除いて２つのやり方に本質的な差はありません。  
コンパイル時に固めたいか、実行時に柔軟に構成を変化させたいかでどちらを採用するか判断すると良いかなと思います。
