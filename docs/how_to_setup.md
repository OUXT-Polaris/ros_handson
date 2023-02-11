# セットアップ方法

本教材では、トラブルを避けるためDockerで環境構築を行います。
そのため、事前にDockerのインストールを済ませておいてください。

Dockerのインストール手順は[こちら](https://docs.docker.com/engine/install/ubuntu/)

!!! note
    Dockerはインストール直後だとsudoをつけないと動かないです。
    インストール後必ず以下のコマンドを実行し、PCの再起動を行ってください
    ```bash
    sudo gpasswd -a $USER docker
    ```


## Docker imageの起動

```bash
docker run --rm -p 6080:80 --shm-size=4096m --security-opt seccomp=unconfined ghcr.io/ouxt-polaris/ros_handson/ros_handson:latest
```

ブラウザで以下のURLを開いてください [http://127.0.0.1:6080/](http://127.0.0.1:6080/)