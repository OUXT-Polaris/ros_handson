# セットアップ方法

本教材では、トラブルを避けるためDockerで環境構築を行います。
そのため、事前にDockerのインストールを済ませておいてください。

## Docker Imageをlocalでビルド

Just run
```
docker build -t ros_handson .
```

## Docker imageの起動

```bash
docker run --rm -p 6080:80 --shm-size=4096m --security-opt seccomp=unconfined ros_handson
```

Browse [http://127.0.0.1:6080/](http://127.0.0.1:6080/)