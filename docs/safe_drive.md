# Rustによるpub/sub通信

## パッケージの作成
すべてのROS2ノードはパッケージに属している必要があります。`ros2 launch`や`ros2 run`コマンド等でROS2ノードを起動する際には必ずパッケージ名が必要になります。 

```bash
# ワークスペースのディレクトリとその配下にパッケージのソースコードを置くディレクトリを作成
mkdir -p /home/ubuntu/Desktop/colcon_ws/src/ros_handson_packages
# ソースコードを置くディレクトリに移動
cd /home/ubuntu/Desktop/colcon_ws/src/ros_handson_packages
# cargo new 
## --libオプションはライブラリクレートを作成するオプションです。
cargo new rust_tutorial --lib
```

## Cargo.tomlの編集
つぎに`Cargo.toml`を編集します。
`/home/ubuntu/Desktop/colcon_ws/src/ros_handson_packages/rust_tutorial/Cargo.toml`を下記のように書き換えてください。

```toml
[package]
# パッケージ名
name = "rust_tutorial"

# パッケージバージョン
version = "0.1.0"

# 使用するRustバージョン
edition = "2021"

[dependencies]
# Rust向けROS2バインディングクレート
safe_drive = "0.2"

# ROS2パラメータ取得ライブラリ
ros2_rust_util = {git = "https://github.com/TakanoTaiga/ros2_rust_util.git"}

# 今回使用するメッセージ
std_msgs = {path = "/tmp/.msg/std_msgs"}

[package.metadata.ros]
# 今回使用するメッセージ
msg = ["std_msgs"]

# メッセージ生成ディレクトリ
msg_dir = "/tmp/.msg/"

# safe_drive バージョン(dependenciesに記載したものと揃える)
safe_drive_version = "0.2"
```

`std_msgs = {path = "/tmp/.msg/std_msgs"}` と `msg_dir = "/tmp/.msg/"`で指定したディレクトリは`/tmp`配下+使用したいメッセージ名であれば問題ありません。`.msg/std_msgs`や `./.msg/`などの相対パスで記載すると一部のメッセージ型ではエラーになる可能性があります。

## package.xmlの追加
つぎに`Cargo.toml`を追加します。
`/home/ubuntu/Desktop/colcon_ws/src/ros_handson_packages/rust_tutorial/package.xml`を追加し下記のように書き換えてください。

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>rust_tutorial</name>
  <version>0.1.0</version>
  <description>safe_serial_bridgeC</description>
  <maintainer email="hoge@hoge.com">Hoge Hogesuke</maintainer>
  <license>Hoge license</license>

  <depend>std_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <export>
    <build_type>ament_cargo</build_type>
  </export>
</package>
```

## pub.rsの追加
`/home/ubuntu/Desktop/colcon_ws/src/ros_handson_packages/rust_tutorial/src`配下に`bin`ディレクトリを作成し
`/home/ubuntu/Desktop/colcon_ws/src/ros_handson_packages/rust_tutorial/src/bin/pub.rs`を追加し下記のように書き換えてください。

```rust
// rustでros2を使えるようにする
use safe_drive::{
    context::Context, error::DynError, logger::Logger, pr_info
};

// パラメータを取得可能にする
use ros2_rust_util::get_str_parameter;


// プログラムをsleepできるようにする
use std::time::Duration;

fn main() -> Result<(), DynError> {
    // コンテキストの作成
    let ctx = Context::new()?;

    // ノードを作成
    let node = ctx.create_node("my_talker", None, Default::default())?;

    // publisherの作成
    let publisher = node.create_publisher::<std_msgs::msg::String>("my_topic", None, true)?;

    // パラメータの取得 
    let param = get_str_parameter(node.get_name(), "name", "saito");


    // ロガーの作成
    let logger = Logger::new("my_talker");

    //カウンター変数
    let mut cnt = 0;

    // pub用メッセージ変数
    let mut msg = std_msgs::msg::String::new().unwrap();

    // 無限ループ
    loop {
        // pubするためのメッセージを作成
        let data = format!("Hello, World!: cnt = {cnt}");

        // Stringデータは直接代入するのではなくassignを使用する
        msg.data.assign(&data);

        // ターミナル出力
        pr_info!(logger, "{:?}-san, send: {}", param, msg.data);

        // publish!
        publisher.send(&msg)?;

        // カウント変数に追加
        cnt += 1;

        // 1秒 sleep
        std::thread::sleep(Duration::from_secs(1));
    }
}
```

## sub.rsの追加
`/home/ubuntu/Desktop/colcon_ws/src/ros_handson_packages/rust_tutorial/src/bin/sub.rs`を追加し下記のように書き換えてください。

```rust
// rustでros2を使えるようにする
use safe_drive::{
    context::Context, error::DynError, logger::Logger, pr_info,
};

fn main() -> Result<(), DynError> {
    // コンテキストの作成
    let ctx = Context::new()?;

    // ノードを作成
    let node = ctx.create_node("my_listener", None, Default::default())?;

    // subscriberの作成
    let subscriber = node.create_subscriber::<std_msgs::msg::String>("my_topic", None, true)?;

    // ロガーの作成
    let logger = Logger::new("my_listener");

    // セレクターの作成
    let mut selector = ctx.create_selector()?;

    // サブスクライバー用callbackを作成
    selector.add_subscriber(
        subscriber,
        Box::new(move |msg| {
            // サブスクライブしたデータを表示
            pr_info!(logger, "receive: {}", msg.data);
        }),
    );

    // 無限ループ
    loop {
        selector.wait()?;
    }
}
```

## パッケージのインストール

下記のコマンドを実行し、パッケージのインストール作業を行なってください。

```bash
# workspaceのルートディレクトリに移動
cd /home/ubuntu/Desktop/colcon_ws
# colcon buildコマンドでrust_tutorialパッケージをインストール
## --symlink-installオプションでファイルをコピーするのではなくシンボリックリンクでインストールするように
colcon build --symlink-install
```

## ros2 runコマンドによるノードの実行

これでノードを実行する準備が整いました。それでは以下のコマンドを立ち上げて作ったROS 2ノードを実行していきましょう。

```bash
source /home/ubuntu/Desktop/colcon_ws/install/local_setup.bash
ros2 run rust_tutorial sub
```

これでsubscribeノードが起動しました。
もう1つターミナルを立ち上げます。

```bash
source /home/ubuntu/Desktop/colcon_ws/install/local_setup.bash
ros2 run rust_tutorial pub
```

２つのコマンドを実行すると、`ros2 run rust_tutorial sub`を実行したターミナルからは下記のような出力が得られます。

```bash
[INFO] [1694838910.442890102] [my_listener]: I heard: "saito-san, send: 0"
[INFO] [1694838910.931468850] [my_listener]: I heard: "saito-san, send: 1"
[INFO] [1694838911.431145869] [my_listener]: I heard: "saito-san, send: 2"
[INFO] [1694838911.931120617] [my_listener]: I heard: "saito-san, send: 3"
[INFO] [1694838912.431568172] [my_listener]: I heard: "saito-san, send: 4"
```

`ros2 run rust_tutorial pub`を実行したターミナルからは下記のような出力が得られます。

```bash
[INFO] [1694838910.442878124] [my_talker]: Publishing: "receive: 0"
[INFO] [1694838910.930593434] [my_talker]: Publishing: "receive: 1"
[INFO] [1694838911.430503775] [my_talker]: Publishing: "receive: 2"
[INFO] [1694838911.930529540] [my_talker]: Publishing: "receive: 3"
[INFO] [1694838912.430752928] [my_talker]: Publishing: "receive: 4"
```

実行を中止するときには`ctrl+c`を押しましょう。

## トラブルシューティング

### Q: ビルド時、書いた覚えのない依存関係を要求されエラーになる
A: 以下のファイルが存在している問題になる場合があります。

- $HOME/.cargo/config.toml
- Cargo.lock (任意のディレクトリ)

`colcon build`を実行したディレクトリに`compile.sh`として以下を記載したシェルスクリプトを実行すると確実で便利です。
```
#!/bin/bash

ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

rm $HOME/.cargo/config.toml
touch $HOME/.cargo/config.toml
rm -rf $ROOT/install/ $ROOT/log/ $ROOT/build/
find $ROOT -name 'Cargo.lock' -type f -exec rm -rf {} \;
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --cargo-args --release
```

### Q: Colcon buildしたのに変更が反映されない

`colcon build`を実行したディレクトリ配下に生成される`install`,`build`,`log`ディレクトリを削除し`colcon build`すると確実に反映されます。

### ros2 topic list や ros2 node listが反応しない

`ps -a`コマンドで作成したノードが起動してないことを確認してください。動作していた場合はパソコンを再起動してください。

### ctrl-c が反応しない

プログラムのミスが原因です。非同期コードを書いている場合に発生することが多いです。シグナル適切に受信するよう`signal-hook-async-std`と`signal-hook`クレートを使うと解決する場合があります。

## 参考情報

- [safe_drive公式チュートリアル](https://tier4.github.io/safe_drive/index.html)
- [safe_driveを使用しマルチスレッドで動作するノード](https://github.com/hakoroboken/safe_smart_controller_gateway.git)
- [multi array型への代入方法](https://github.com/hakoroboken/safe_smart_controller_gateway/blob/7ea2b217ca4ad54b7cfda3751bab76e2e0d2bc2e/src/network_module.rs#L100)


## このチュートリアルに記載されているコードについて
[safe_drive公式チュートリアル](https://tier4.github.io/safe_drive/index.html)に記載されているコードにライセンスを確認の上、一部加筆したものを掲載しています。
