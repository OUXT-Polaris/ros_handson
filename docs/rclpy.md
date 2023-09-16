# Pythonによるpub/sub通信

## パッケージの作成

すべてのROS2ノードはパッケージに属している必要があります。`ros2 launch`や`ros2 run`コマンド等でROS2ノードを起動する際には必ずパッケージ名が必要になります。  
本教材で出来上がるコードは[こちらのリポジトリ](https://github.com/OUXT-Polaris/ros_handson_packages.git)に収録しています。参考にしてみてください。  
また、コードの詳細な解説はコードのコメントに残してあります。ぜひご参照ください。  
`ros2 pkg create`コマンドを使用し、実際にパッケージを作ってみましょう。  

```bash
# ワークスペースのディレクトリとその配下にパッケージのソースコードを置くディレクトリを作成
mkdir -p /home/ubuntu/Desktop/colcon_ws/src/ros_handson_packages
# ソースコードを置くディレクトリに移動
cd /home/ubuntu/Desktop/colcon_ws/src/ros_handson_packages
# ros2 pkg createコマンドを使用してパッケージを作成する
## --dependenciesオプションは依存するパッケージを指定することができます。
## --build-typeオプションはビルドの仕方を指定します。デフォルトはament_cmakeなのでこれを忘れるとC++のパッケージが作られます。
## --licenseオプションはライセンスを指定します。なんらかの論文などのコードを公開するときにはライセンスがついていると親切です。
ros2 pkg create python_tutorial --dependencies rclpy --build-type ament_python --license Apache-2.0
```

`--dependencies`オプションで指示した結果は`package.xml`に反映されます。
ROSのビルドシステムであるcolconは`package.xml`の依存を確認してビルド順を制御、適切に環境を構築していきます。

`--license`オプションの指定がない場合、パッケージにライセンスが振られません。
ここを適切につけておかないと結果みなさんが作ったパッケージを他の人が使っていいか判断ができなくなってしまいます。
