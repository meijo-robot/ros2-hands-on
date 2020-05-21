## 0-1.仮想環境の構築

ROS2をインストールするために、Ubuntuの仮想環境を構築します。

### 0-1-1.OracleVirtualBoxのインストール

下記リンクよりVirtualBoxをインストールしてください

https://www.virtualbox.org/


### 0-1-2.VirtualBoxにUbuntu **18.04** をインストールする

ubuntu のインストールは下記のQiita等を参照下さい
- https://qiita.com/yoshi0518/items/85549e93e1d4eb6d0a12#sshd
- https://qiita.com/pyon_kiti_jp/items/0be8ac17439abf418e48
- https://www.sejuku.net/blog/82291

インストール後にターミナルを起動して以下のコマンドを実行してください。
``` sh
$ sudo apt update
$ sudo apt upgrade
```


VirtualBoxの**Guest Addition CD** をインストールして仮想OSを再起動してください

![](./images/6_GuestAdditions.png)


下記のコマンドでgitのインストールをしてください
```sh
$ sudo apt install git 
```

gitは、ソースコードなどの変更履歴を記録して管理するバージョン管理システムと呼ばれるものです。 今回のセミナーでは詳細は触れませんが、研究開発を行う上では非常に有用なシステムですので、利用をお勧めします。 公式の解説書、[Pro Git](https://git-scm.com/book/ja/v2)などを参考にして下さい


必須ではありませんが、
VSCodeをインストールしておくとコードの入力間違い等が少なく便利です
```
$ sudo snap install --classic code
```

これでOSのインストールは終了です。

## 0-2.ROS2のインストール

ここからはROS2のインストールをしていきます。

### 0-2-1.OSの文字コードをUTF-8に設定し、ロケールを変更する

```sh
$ sudo locale-gen ja_JP ja_JP.UTF-8
$ sudo update-locale LC_ALL=ja_JP.UTF-8 LANG=ja_JP.UTF8
$ export LANG=ja_JP.UTF-8
```

### 0-2-2.リストに追加して鍵を許可する

公式aptパッケージのダウンロード先をaptソースリストに追加する。
ROS2のパッケージリポジトリをaptのsources.list.dに追加し、リポジトリのGPG鍵を許可する。

```sh
$ sudo apt update
$ sudo apt install curl gnupg2 lsb-release
$ curl http://repo.ros2.org/repos.key | sudo apt-key add -
```

### 0-2-3.リポジトリの登録
下記のコマンドで追加のリポジトリをROS2に追加します
```sh
$ sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
$ sudo apt update
```

### 0-2-4.全てのROS2パッケージのインストールを行う

下記のコマンドで、ROS2パッケージのインストールを行います

```sh
$ export ROS_DISTRO=dashing
$ sudo apt update
$ sudo apt install ros-${ROS_DISTRO}-desktop
```

### 0-2-5.Pythonのargcompleteパッケージ他のインストール
下記のコマンドで、ROS2で使うビルドシステムなどのソフトウェアをインストールします

```sh
$ sudo apt install ros-$ROS_DISTRO-desktop python3-colcon-common-extensions python3-rosdep python3-argcomplete
```
- ros-$ROS_DISTRO-desktop 
  - ROS2本体(GUI,他のツール等を全て含みます)
- python3-colcon-common-extensions
  - ROSパッケージをビルドするためのツール
- python3-rosdep
  - ROSパッケージの依存関係解決を行うツール

### 0-2-6.PATHを通す

ROSを使用する際には、/opt/ros/以下のフォルダにパスが通っていなければなりません。~/.bashrcにパス追加のコマンドを追加しておきましょう。

```sh
$ echo "source /opt/ros/dashing/setup.bash" >> ~/.bashrc
```

## 0-3.ROS2のインストール確認

ROS2のインストールが正常に行えているかどうかをサンプルプログラムを用いてチェックを行います。
ターミナルを立ち上げ、
```
$ ros2 run demo_nodes_cpp listener
```
と入力してlistenerノードを起動してください。

 **別のターミナル**を立ち上げ、
```
$ ros2 run demo_nodes_cpp talker
```

と入力してtalkerノードを起動してください。
それぞれのターミナルに同様の文字が出力されていれば成功です。

