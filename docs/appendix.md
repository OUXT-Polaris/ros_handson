# 付録


## ポーティングしやすいROS1ノード実装方法

### main文とロジックを分離する
ROS2でノードを実装する際にはmain文があるnodeとロジックだけを実装するコンポーネントに分けておくと、コンポーネント指向でROS2システムをつくることが簡単にできます。
ROS1でも同じように[こちらのサンプル](https://github.com/OUXT-Polaris/nmea_to_geopose/tree/master/src)
にあるようにmain文とロジックを実装したクラスを分けておくと、コンポーネント指向なノードにポーティングする際に非常に楽です。

そして、ロジックを実装するクラスのコンストラクタでは、ROS2のコンポーネントと同じようにコンストラクタで
```cpp
NmeaToGeoPose::NmeaToGeoPose(ros::NodeHandle nh,ros::NodeHandle pnh)
{
   nh_ = nh;
   pnh_ = pnh;
   pnh_.param<std::string>("input_topic", input_topic_, "/nmea_sentence");
   nmea_sub_ = nh_.subscribe(input_topic_,10,&NmeaToGeoPose::nmeaSentenceCallback,this);
   geopose_pub_ = pnh_.advertise<geographic_msgs::GeoPoseStamped>("geopose",1);
}
```
publisher/subscriberを作成しておくとさらにポーティングが容易になります。
上記のコードは[こちら](https://github.com/OUXT-Polaris/nmea_to_geopose/blob/2564e99b65418ab9ba216b5664601e51ca53e6ec/src/nmea_to_geopose.cpp#L3-L10)
にサンプルがあります。

### メッセージのパッケージは独立させておく

ROS1ではロボットのアプリケーションロジックを含むパッケージに.msgファイルを置いておいても全く問題なかったのですが、
ROS2においてはメッセージパッケージを分けないとビルドができなくなるケースがあります。
そのため、ROS1でもメッセージのパッケージを分離しておくとROS2に移住する際に楽かと思います。

### Executorが使える言語で実装しておく

現在いくつかのROS2クライアントにはさまざまな種類が存在しています。
しかし、そのなかでExecutorが実装されて居るのは一部のみです。

| クライアント名 | 言語   | URL                                        | Executorの実装 |
| -------------- | ------ | ------------------------------------------ | -------------- |
| rclcpp         | C++    | https://github.com/ros2/rclcpp             | あり           |
| rclpy          | python | https://github.com/ros2/rclpy              | あり           |
| rclc           | C      | https://github.com/ros2/rclc               | あり           |
| rclnodejs      | nodejs | https://github.com/RobotWebTools/rclnodejs | なし           |
| rust           | rust   | https://github.com/ros2-rust/ros2_rust     | なし           |
| rclgp          | go     | https://github.com/juaruipav/rclgo         | なし           |

開発者がROS2公式に近いクライアントではあらかたexecutorの実装は行なわれていますが、それ以外のクライアントではあまり実装されていないなという印象です。
実装に使用したい言語でROS2クライアントがあるかどうか、そのクライアントにExecutor実装があるかを事前に調査して置くとパフォーマンスの高いROS2アプリケーションを
実装することが可能です。

## パフォーマンスを上げるには

ロボットはいわゆるリアルタイムシステムであり、事前に設計した時間までに処理を終わらせていかなければなりません。
そのためにはパフォーマンスを出せるように意識しながらシステム全体を設計していくことが必要です。
### どのExecutorの上でどのノードが動いているかを意識する

[以前の章](https://hakuturu583.github.io/ros_rsj_seminar/ros2/#nodeexecutor)で示したようにExecutor間で画像や点群といった大容量のデータを通信してしまうと
大きなレイテンシの原因になりますし、参考文献[1]の発表の図表を見るとComponentを使わなければROS1よりパフォーマンスが劣化してしまう可能性があります。
そのため、ROS2でシステムを設計する際には広い通信帯域が必要なトピックを同じExecutorに乗せるのを意識することが重要です。
特に分散計算機環境に於いては、「どの計算機でどの処理を行なうのか」「どうExecutorに乗せると通信待機の消費が少なくなるか」をよく考えながらシステム全体を設計していく必要があります。

### 実装する言語を揃える

Pythonで実装されているノードとC++で実装されているノードを同じExecutor上で実行することは原理的に不可能です。
そのため、実装する言語をある程度揃えておかないとそこがボトルネックになりシステム全体のパフォーマンスに悪影響を及ぼします。

### DDSを選定し、設定を工夫する
ROS2にはさまざまなDDSが存在します。
ROS1時代には通信層を交換することができずほとんどチューニングできませんでしたが、ROS2からは
自分のアプリーケーションに合わせてDDSを選定したり、DDSの設定を見直すだけで大きくパフォーマンスは向上します。

例えば、固定長配列しか用いない、かつ一台のマシンでシステムが完結するのであればzero-copy転送をうたうIceOryxは非常に強力な選択肢になります。
自分の計算機環境に合わせて適切なDDSを選定し、DDSのドキュメントを読んで適切な設定をしてみましょう。

ちなみに、参考までに筆者の開発環境では以下のような設定をしています。

/opt/masaya/cyclonedds_config.xmlに以下のxmlを保存

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain id="any">
    <General>
      <NetworkInterfaceAddress>lo</NetworkInterfaceAddress>
    </General>
    <Internal>
      <MinimumSocketReceiveBufferSize>10MB</MinimumSocketReceiveBufferSize>
    </Internal>
  </Domain>
</CycloneDDS>
```

起動時に毎回このスクリプトを実行
```bash
export CYCLONEDDS_URI=file:///opt/masaya/cyclonedds_config.xml
sudo sysctl -w net.core.rmem_max=2147483647 # 受信用ウィンドウサイズの上限値を指定
sudo ifconfig lo multicast # マルチキャストをローカルネットワークで有効化
```
