# Windows での Docker のセットアップ

windows 10でのdockerのインストール方法についての簡単な説明です。

## 1. Docker Desktop for Windows のインストール

https://docs.docker.jp/docker-for-windows/install.html
を参照してDockerDesktopをインストールしましょう。

![Docker docs](images/docker_install_for_windows/1.PNG)

Docker Hubからダウンロードをクリック

![インストール先](images/docker_install_for_windows/2.PNG)

ダウンロードしたらこんな感じです
。
![ダウンロードしたフォルダ](images/docker_install_for_windows/3.PNG)

Docker Desktop Installer.exeをクリックするとインストーラーが立ち上がります。

![インストーラー起動後](images/docker_install_for_windows/4.PNG)

OKを押して進めましょう。

![インストール中](images/docker_install_for_windows/5.PNG)

![インストール終わり](images/docker_install_for_windows/6.PNG)

ここまで来たら再起動しましょう。

再起動後、こんな画面が出ると思います。

![同意画面](images/docker_install_for_windows/7.PNG)

Acceptを押したら完了です。

もしかしたらwslのアップデートをしてくれと言われる場合も有ります。そしたら、PowerShellに

```
wsl --update
```

を入力してアップデートしましょう。
![powershell](images/docker_install_for_windows/8.PNG)

Docker Desktopが起動するとこんな感じの画面になります。

![DockerDesktop](images/docker_install_for_windows/10.PNG)

# イメージのダウンロード

Docker imageをダウンロードします。

```
docker run --rm -p 6080:80 --shm-size=4096m --security-opt seccomp=unconfined ghcr.io/ouxt-polaris/ros_handson/ros_handson:latest
```

このコマンドをコピーしてPowerShellに入力してください。
![Image](images/docker_install_for_windows/11.PNG)
![Powershell](images/docker_install_for_windows/12.PNG)

ダウンロードが始まるとこんな感じです。
![ダウンロード中](images/docker_install_for_windows/13.PNG)
![起動後](images/docker_install_for_windows/14.PNG)
ダウンロードが終わるとイメージが起動して、127.0.0.1:6080に接続できるようになります。

gazeboを起動するためのターミナルはここにあります。
![ターミナル](images/docker_install_for_windows/16.PNG)

# その他

イメージをストップするにはDocker Desktopの画面で停止アイコンを押します。
![停止方法](images/docker_install_for_windows/17.PNG)

もう一度起動するには再度PowerShellで

```
docker run --rm -p 6080:80 --shm-size=4096m --security-opt seccomp=unconfined ghcr.io/ouxt-polaris/ros_handson/ros_handson:latest
```

を入力すれば良いです。起動にはちょっと時間がかかります。

# 参考文献

https://github.com/AI-Robot-Book/chapter1/tree/main/docker
