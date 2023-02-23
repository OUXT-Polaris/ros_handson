# セットアップ方法

本教材では、トラブルを避けるため Docker で環境構築を行ないます。
そのため、事前に Docker のインストールを済ませておいてください。

Docker のインストール手順は[こちら](https://docs.docker.com/engine/install/ubuntu/)

Windows での Docker のインストールは[こちら](docker_install_for_windows.md)

!!! note
Docker はインストール直後だと sudo をつけないと動かないです。
インストール後必ず以下のコマンドを実行し、パソコンの再起動を行なってください。

    ```
    sudo gpasswd -a $USER docker
    ```

    以下のコマンドを入力して

    ```
    docker run hello-world
    ```

    `Hello from Docker!`を含む出力が得られていればDockerのインストールは正常に完了しています。

!!! warning
本教材を執筆するにあたって筆者が使用している環境は`Ubuntu22.04`です。  
 Docker が動けば他の OS でも動く可能性はあるとは思います。  
 ただし M1 Mac などの非 x86 系 CPU の上だと多分動かないです。

Docker はソフトウェアの動作似必要なライブラリ群を 1 つにまとめ、共有できる仕組みです。  
あたかも仮想マシンのように(厳密にはコンテナ仮想化といいます)振る舞い、`docker pulll`や`docker run`といったコマンドを叩くだけでかんたんに環境を再現できます。　　
このとき、動作環境に必要なファイルを 1 つにまとめたものを「Docker Image」、そのファイルから作られた動作環境のことを「Docker Container」といいます。
1 つの Docker Image から複数の Docker Container を作って同時に動かすことも可能です。
Docker Image が本体、Docker Container が分身という関係性あると考えるとわかりやすいかもしれません。

## Docker image の起動

Docker Image は github actions というツールを使用して自動的にビルドされ github container registry においてあります。
そのためビルドする必要はありません。
github actions は github で開発を行なうときに使用できる非常に強力な CI/CD ツールで皆さんの競技活動、研究活動を大きく効率化してくれますので、付録に使い方を載せておきます。

```bash
docker run --rm -p 6080:80 --shm-size=4096m --security-opt seccomp=unconfined ghcr.io/ouxt-polaris/ros_handson/ros_handson:latest
```

ブラウザで以下の URL を開いてください [http://127.0.0.1:6080/](http://127.0.0.1:6080/)
VNC が立ち上がるので、ブラウザから Docker Conteiner 内部の環境にアクセスできます。

![Not Found](images/desktop.png)

こちらの環境にはすでに必要なソフトウェアがインストールされていますので、vnc 上でシェルを立ち上げます。

```bash
gazebo
```

立ち上げたターミナルに上記のコマンドを入力してみてください、すると gazebo が立ち上がります。

![Not Found](images/launch_gazebo.png)

!!! warning
円滑な開催のため、ハンズオン会場に来られる前にここまでは必ず動作確認をお願いします。

今回はこの環境使ってハンズオンを進めていこうとおもいます。

## VScode のインストール

こちらはマストではありませんが、あれば非常に楽です。  
今回のハンズオンでは基本的に VNC 上で操作は完結しますが、Docker Container に接続しデータをコピーしたりできるため、[VSCode](https://azure.microsoft.com/ja-jp/products/visual-studio-code)
および[DevContainer プラグイン](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)はあったほうが何かと便利かと思われます。
