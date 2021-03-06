# 0.事前準備

## 0-1.仮想環境の構築

ROS2をインストールするために、Ubuntuの仮想環境を構築します。

### 0-1-1.OracleVirtualBoxのインストール

下記リンクよりVirtualBoxをインストールしてください

[https://www.virtualbox.org/](https://www.virtualbox.org/)

### 0-1-2.VirtualBoxにUbuntu **18.04** をインストールする

ubuntu のインストールは下記のQiita等を参照下さい

- [VirtualBoxへ Ubuntu 18.04 をインストール -- Qiita](https://qiita.com/yoshi0518/items/85549e93e1d4eb6d0a12#sshd)
- [Windows10にVirtualBoxとUbuntuをインストール -- Qiita](https://qiita.com/pyon_kiti_jp/items/0be8ac17439abf418e48)

インストール後にターミナルを起動して以下のコマンドを実行してください。

```shell
$ sudo apt update
$ sudo apt upgrade
...(略)
```

VirtualBoxの**Guest Addition CD** をインストールして仮想OSを再起動してください

VirtualBoxのゲストOS起動中に[デバイス]->[Guest Addition CDイメージの挿入]を選択すると、インストール用の仮想ディスクがゲストOSに挿入されます。

![gust_addition_CD](img/guest_addition_.jpg)

以下の画面が出るので[実行する]をクリックしてください。

![認証](img/auth.jpg)

認証を求められるのでパスワードを入力してください。インストールが完了すると再起動を求められるので、ゲストOSを再起動してください。

GuestAddtionをインストールすることで、ゲスト-ホストOS間でのクリップボードの共有や、画面サイズの自動変更が可能になります。

## 0-1-3.ハンズオンで使用するソフトウェアをインストールする

再起動後、下記のコマンドでgitのインストールをしてください

```shell
sudo apt install git
```

gitは、ソースコードなどの変更履歴を記録して管理するバージョン管理システムと呼ばれるものです。 今回のセミナーでは詳細は触れませんが、研究開発を行う上では非常に有用なソフトですので、利用をお勧めします。 使い方は公式の解説書、[Pro Git](https://git-scm.com/book/ja/v2)などを参考にして下さい

また、必須ではありませんが、VSCodeをインストールしておくとコードの入力間違い等が少なく便利です

以下のコマンドでインストールできます。

```shell
sudo snap install --classic code
```

これでOSと関連ソフトウェアのインストールは終了です。

## 0-2.ROS2のインストール

ここからはROS2のインストールをしていきます。

### 0-2-1.OSの文字コードをUTF-8に設定し、ロケールを変更する

```sh
$ sudo locale-gen ja_JP ja_JP.UTF-8
$ sudo update-locale LC_ALL=ja_JP.UTF-8 LANG=ja_JP.UTF8
$ export LANG=ja_JP.UTF-8
(略)
```

### 0-2-2.リストに追加して鍵を許可する

公式aptパッケージのダウンロード先をaptソースリストに追加する。
ROS2のパッケージリポジトリをaptのsources.list.dに追加し、リポジトリのGPG鍵を許可する。

```sh
$ sudo apt update
$ sudo apt install curl gnupg2 lsb-release
$ curl http://repo.ros2.org/repos.key | sudo apt-key add -
(略)
```

### 0-2-3.リポジトリの登録

下記のコマンドでROS2のリポジトリをUbuntuに追加します

```sh
$ sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
$ sudo apt update
(略)
```

### 0-2-4.全てのROS2パッケージのインストールを行う

下記のコマンドで、ROS2パッケージのインストールを行います

```sh
$ export ROS_DISTRO=dashing
$ sudo apt update
$ sudo apt install ros-${ROS_DISTRO}-desktop
(略)
```

### 0-2-5.Pythonのargcompleteパッケージ他のインストール

下記のコマンドで、ROS2で使うビルドシステムなどのソフトウェアをインストールします

```sh
$ sudo apt install python3-colcon-common-extensions python3-rosdep python3-argcomplete
(略)
```

- python3-colcon-common-extensions
  - ROSパッケージをビルドするためのツール
- python3-rosdep
  - ROSパッケージの依存関係解決を行うツール

### 0-2-6.PATHを通す

ROSを使用する際には、/opt/ros/以下のディレクトリにパスが通っていなければなりません。~/.bashrcにパス追加のコマンドを追加しておきましょう。

```sh
$ echo "source /opt/ros/dashing/setup.bash" >> ~/.bashrc
(略)
```

## 0-3.ROS2のインストール確認

ROS2のインストールが正常に行えているかどうかをサンプルプログラムを用いてチェックを行います。
ターミナルを立ち上げ、以下を入力してlistenerノードを起動してください。

```shell
$ ros2 run demo_nodes_cpp listener
[INFO] [listener]: I heard: [Hello World: 1]
[INFO] [listener]: I heard: [Hello World: 2]
...
```

listenerノードの起動後、 **別のターミナル**を立ち上げ、

```shell
$ ros2 run demo_nodes_cpp talker
[INFO] [talker]: Publishing: 'Hello World: 1'
[INFO] [talker]: Publishing: 'Hello World: 2'
....
```

と入力してtalkerノードを起動してください。

それぞれのターミナルに同様の文字が出力されていれば成功です。

確認できたら各ターミナルで[Ctrl + c]を押してプログラムを停止してください。
