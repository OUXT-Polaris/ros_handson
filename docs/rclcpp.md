# 自作ノードの作り方(C++)

## パッケージの作成

すべてのROS2ノードはパッケージに属している必要があります。`ros2 launch`や`ros2 run`コマンド等でROS2ノードを起動する際には必ずパッケージ名が必要になります。  
本教材で出来上がるコードは[こちらのリポジトリ](https://github.com/OUXT-Polaris/ros_handson_packages.git)に収録しています。参考にしてみてください。  
`ros2 pkg create`コマンドを使用し、実際にパッケージを作ってみましょう。  

```bash
ros2 pkg create tutorial --dependencies rclcpp ament_cmake_auto geometry_msgs --library-name publish --license Apache-2.0
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

パッケージを作った段階では、ament_cmake_autoは使われていません、このままではとても記述が面倒なのでCMakeLists.txtを以下のように書き換え、ament_cmake_autoを使ってみましょう。

<details>
<summary>CMakeLists.txt</summary>
```CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(subscribe src/subscribe.cpp)
ament_auto_add_library(publish src/publish.cpp)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
```
</details>

だいぶスッキリしたかと思います。  
では、次に共有ライブラリ側にPublisher/Subscriberを実装し、Pub/Sub通信をしていきたいと思います。  

## Publisher/Subscriberの実装
tutorialパッケージのincludeディレクトリに以下のsubscriber.hppを置きます。  

```C++

```
