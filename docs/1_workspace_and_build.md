# 1. ワークスペースの作成とビルド方法

ここではワークスペースの作成方法とcolconを使用したビルド方法についての解説を行います。

## 1-1. ワークスペースの構造

ずは、ワークスペースの構造を理解するために作成してみましょう。
下記コマンドを実行してください

```sh
source /opt/ros/dashing/setup.bash
mkdir -p ~/intro_colcon/src
```

次にgitを使って練習用のパッケージをダウンロードします。

```sh
cd ~/intro_colcon/src
git clone https://github.com/ros2/examples.git
cd examples
git checkout dashing
```

src/下にソースコードがダウンロードされるので、ディレクトリ構造などを色々覗いてみましょう。

## 1-2 colconを使用してビルドする

git でダウンロードしたパッケージをビルドしていきましょう。

ビルドには**colcon**というコマンドを使います。
colconはワークスペースを扱うときにワークスペースの最上位ディレクトリ(src/が見えるディレクトリ)で実行しなければなりません。

-h のオプションをつけることで、各サブコマンドのヘルプを確認することができます。
やってみましょう。

```sh
$ colcon -h
usage: colcon [-h] [--log-base LOG_BASE] [--log-level LOG_LEVEL]
              {build,extension-points,extensions,info,list,metadata,test,test-result,version-check}
              ...
[略]
```

まずはbuildサブコマンドの使い方をチェックしましょう

``` sh
$ colcon build -h
usage: colcon build [-h] [--build-base BUILD_BASE]

[略]
```

それではビルドししょう

``` sh
cd ~/intro_colcon
colcon build
[略]
```

今回は初回ビルドなので、すべてのソースをまとめてビルドしましたが、以下のコマンドでパッケージを指定してビルドすることができます。

``` sh
cd ~/intro_colcon
colcon build --packages-select examples_rclcpp_minimal_client
```

lsコマンドでsrc/ディレクトリを見てみましょう。以下の3つのディレクトリが新しくできていれば成功です。

- build
- install
- log

ビルドが完了したら、実行してみましょう。
ROSのプログラムを実行する際にはまず、端末でそのワークスペースをアクティベートが必要です。ワークスペースの直下で```source install/local_setup.bash```を実行しましょう。
注意しなければならないのは、このコマンドで有効化できるのは単一の端末だけです。新しい端末を開いた場合は同じコマンドを実行しなければなりません。一つの端末でそれぞれ一回だけ実行すれば十分ですが、ROSの開発では複数の端末を開くことが多く、忘れやすいので気をつけてください。

また、ビルド時に新しいパッケージをsrcに追加した場合にも再度実行されなければなりません。

``` sh
source install/local_setup.bash
```

プログラムは下記のコマンドで実行可能です。

``` shell
$ ros2 run examples_rclcpp_minimal_publisher publisher_member_function
[INFO] [minimal_publisher]: Publishing: 'Hello, world! 0'
[INFO] [minimal_publisher]: Publishing: 'Hello, world! 1'
[INFO] [minimal_publisher]: Publishing: 'Hello, world! 2'
[INFO] [minimal_publisher]: Publishing: 'Hello, world! 3'
[INFO] [minimal_publisher]: Publishing: 'Hello, world! 4'
....
```

## 1-5.ColconIgnore

パッケージによってはcolconでビルドしたくないパッケージがあると思います。パッケージのディレクトリに"COLCONIGNORE"というタイトルのファイルを作るだけで、そのパッケージはビルド時に無視されます。

(1.3〜1.4は参考です。)

## 1-3. (参考)symlinkインストール

colconはcatkinと違って、すべてのビルドされたファイルがソフトリンクされたdevelスペースがありません。 ですので、開発中のパッケージをインストールせずに利用することはできません。 ROS1の時代ではdevelスペースを利用して、インストールをせずに使用することができましたが、ROS２にはありません。代わりに「symlink install」という手法を利用します。 これはインストールステップ実行時に、ファイルをコピーするのではなくソフトリンクを作成します。 結果的にdevelスペースのようになりますが、各ビルドツールのインストール機能を利用しているのでパッケージの開発者は何がどこにソフトリンクされるか管理できます。

symlinkインストールを行うにはには--symlink-installオプションをつけてビルドしてください。

```sh
cd ~/intro_colcon
colcon build --symlink-install
```

## 1-4. (参考)テストの実行

テスト含まれているパッケージがあれば以下のコマンドでテストが実行可能です。

テストの実行

```sh
cd ~/intro_colcon
colcon test
```

[2-1へ進む](2_1_ROS2_API.md)
