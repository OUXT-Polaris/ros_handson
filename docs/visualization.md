# 可視化

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
