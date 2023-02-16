# 可視化

## laserscanの可視化

遠隔操作でロボットを動かすことはできるようになりましたが、ここまで紹介した方法ではロボットからどういうセンサデータを受信しているかは`ros2 topic echo`などのコマンドを見るしかありません。
しかし、それではカメラやLiDARといった高い次元数を持つデータを正しく送られているかを確認することができません。
そこで、`rviz2`という3Dビジュアライザを使ってgazeboによってシミュレートされたセンサ情報を表すトピックを可視化してみましょう。

手順としては、まず以下のコマンドを実行し、gazeboを起動します。

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

つぎに以下のコマンドを実行し、`rviz2`を起動します。

```bash
rviz2
```

rviz2を起動すると、`Frame [map] does not exist`というエラーが赤字で表示されていると思います。
いきなりエラーとは何事か？と思われるかもしれませんが、これは正常な動作です。
rviz2の描画の基準になる座標系（Fixed Frame）はデフォルトでは`map`という名前になっていますが、まだ`map`座標系を計算するノードはありません。
そのためこの状況はrviz2上ではエラーにはなっているものの、システムの挙動としては正常という状態になります。
しかしエラー状態では何も描画できないので、Fixed Frameのドロップダウンリストから`base_link`の選択肢を選びます。

次にDisplayパネルのAddをクリックし、By Topicパネルを開きます。
するとそこには現在発行されているトピックとそれに対応する可視化プラグインの一覧があります。
今回はLiDARのscanデータを可視化したいので、scanを選択してください。
下の動画に示すような赤い点群が得られたら成功です。

<iframe width="1280" height="720" src="https://www.youtube.com/embed/NMMXNz6CXwQ" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

もし、赤い点群が得られたらDisplayパネルから可視化の設定をいじって点群の色を変えてみたり、大きさや描画方法を変更してみてください。

さらに、[前章](teleop.md)で示した遠隔操作ノードと組み合わせるとロボットの動きに合わせてLiDARの計測結果も変化しているのがわかります。

<iframe width="1280" height="720" src="https://www.youtube.com/embed/AjXtOwsTNlg" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

## ログデータの可視化

ロボット、特に重量のある大型ロボットは試験が非常に大変です。
例えばOUXT Polarisが競技に使用しているWAM-Vは組み立てに4人がかりで2~3時間程度必要です。
そんなロボットを実験する機会というのは非常に貴重であり、開発スケジュールの遅れに繋がります。
そこで重要な役割を担うのがロギングツールです。
ROS2ではrosbag2というロギングツールが採用されています。

<blockquote class="embedly-card"><h4><a href="https://github.com/ros2/rosbag2">GitHub - ros2/rosbag2</a></h4><p>Repository for implementing rosbag2 as described in its corresponding design article. rosbag2 packages are available via debian packages and thus can be installed via $ export CHOOSE_ROS_DISTRO=crystal # rosbag2 is available starting from crystal $ sudo apt-get install ros-$CHOOSE_ROS_DISTRO-ros2bag ros-$CHOOSE_ROS_DISTRO-rosbag2* Note that the above command installs all packages related to rosbag2.</p></blockquote>
<script async src="//cdn.embedly.com/widgets/platform.js" charset="UTF-8"></script>

rosbag2はデフォルトではsqlite3という形式でデータを保存しますが、性能やcdrに存在する非常につよい制約の関係上mcapという形式を推奨します。
mcapを使うためには`rosbag2_storage_mcap`パッケージのインストールが必要です。
ハンズオン環境にはすでに入っていますので追加は不要ですが、もし今後お手元のロボットで使用するときは`rosbag2_storage_mcap`のインストールを忘れないようにしましょう。

ハンズオン環境でgazeboの立ち上げを行った後、新たにターミナルを立ち上げ、以下のコマンドを実行してrosbagのデータを保存します。
`--storage mcap`の指定を忘れるとデフォルトの形式であるsqlite3で保存が行われてしまうので気をつけましょう。

```bash
ros2 bag record -a --storage mcap
```

もうロギングはいいかな、となったところで`ctrl+C`でrecordを終了してください。

<iframe width="1280" height="720" src="https://www.youtube.com/embed/XHF1vQSpPuo" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
