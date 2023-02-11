# 遠隔操作

まずは一番シンプルなキーボード入力による遠隔操作を試してみましょう。

## 操作手順
VNCでハンズオン環境に入り、以下のコマンドを実行します。

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

すると、gazeboのワールドが立ち上がります。
そしてもう一つのターミナルを立ち上げ以下のコマンドを入力します。

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

上のコマンドを実行したターミナルに以下の出力がなされた場合、うまく動いています。

```bash
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key, s : force stop
```

w/xを押すとTurtlebot3が加減速、a/dを押すと左右旋回します。sを押すと緊急停止します。

<iframe width="1280" height="720" src="https://www.youtube.com/embed/CWrNiMq1AWo" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

