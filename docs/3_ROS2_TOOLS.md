# 3 コマンド

## 3-1.「ros2」コマンド

ros2はROS 2の主なコマンドライン（CUI）ツールです。 ROS1のrostopic、rosservice等のコマンドの代替です。 様々な「verb」と呼ぶサブコマンドでROS2のノード、トピック等を操ることができます。

各サブコマンドは--helpオプションをつけることで、詳細なヘルプをみることができます。トピックやノードの

```shell
$ ros2 -h
usage: ros2 [-h] Call `ros2 <command> -h` for more detailed usage. ...

ros2 is an extensible command-line tool for ROS 2.

optional arguments:
  -h, --help            show this help message and exit

Commands:
  action     Various action related sub-commands
  component  Various component related sub-commands
  daemon     Various daemon related sub-commands
  launch     Run a launch file
  lifecycle  Various lifecycle related sub-commands
  msg        Various msg related sub-commands
  multicast  Various multicast related sub-commands
  node       Various node related sub-commands
  param      Various param related sub-commands
  pkg        Various package related sub-commands
  run        Run a package specific executable
  security   Various security related sub-commands
  service    Various service related sub-commands
  srv        Various srv related sub-commands
  topic      Various topic related sub-commands

  Call `ros2 <command> -h` for more detailed usage.
```

パート2までに出てこなかったサブコマンドを解説します。

## 3-2.「pkg」サブコマンド

パッケージの関連のサブコマンドですlistでパッケージの一覧を見ることができます。また、createを使用するとパッケージの作成が可能です。

```shell
$ ros2 pkg list
action_msgs
action_tutorials
略
$ cd somewhere_workspace
....
$ ros2 pkg create test
# パッケージの作成
going to create a new package
package name: test
...略
```

## 3-3.「launch」サブコマンド

複数のROSノードの起動をまとめて行いたいときなどにはlaunchファイルを書くことができます。ROS1ではXMLによる記述しかできませんでしたが、
ファイルを読み込んで起動条件を変えたり、[起動中のイベントを判断して反応するlaunchファイル](https://github.com/ros2/launch/blob/master/launch/examples/launch_counters.py)なども実装可能です。

試しにlaunchファイルを書いてみましょう。

2_1_sending_message/greeting/にlaunchディレクトリを作成して、以下のファイルtest.launch.pyを追加して下さい。

```python 2_1_sending_message/greeting/launch/test.launch.py
import launch
import launch_ros.actions

def generate_launch_description():
    greeter = launch_ros.actions.Node(
        package='greeting',
        node_executable='greeter',
        output='screen'
        )
    displayer = launch_ros.actions.Node(
        package='displayer_basic_version',
        node_executable='displayer',
        output='screen'
        )

    return launch.LaunchDescription([
        greeter,
        displayer,
        ])
```

また、greeterパッケージのCMakeLists.txtの末尾に以下を追加してください

```CMAKE CMakeLists.txt

# ####ここから追加#####
# launchディレクトリをshareへインストールする
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})
# ####ここまで###

# ## 末尾のament_package()より上に追加する ##
# amentのリソースインデクスにパッケージを登録する
ament_package()
```

使用するときは以下のコマンドで使います。

```shell
$ . install/local_setup.bash
$ ros2 launch greeting test.launch.py
[INFO] [launch]: All log files can be found below /home/takumi/.ros/log/2020-05-22-23-02-45-016548-takumi-VirtualBox-12086
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: launch greeter and displayer node
[INFO] [greeter-1]: process started with pid [12104]
[INFO] [displayer-2]: process started with pid [12105]
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[greeter-1] [INFO] [greeter]: Publishing greeting 'hello world'
[greeter-1] [INFO] [greeter]: Publishing greeting 'hello world'
(略)
```

ログの出力がバッファに溜まるまでターミナルには出力されないため、[ctrl + c]で終了シグナルを送ると、バッファの出力が表示ターミナルに表示されます。

[4.作ってみよう に進む](4_Turtle.md)
