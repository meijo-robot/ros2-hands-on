# 3 コマンド

## 3-1.「ros2」コマンド

ros2はROS 2の主なコマンドライン（CUI）ツールです。 ROS 1のrostopic、rosservice等のコマンドの変わりです。 様々な「verb」と呼ぶサブコマンドでROS 2のノード、トピック等を操ることができます。

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
$ cd workspace
....
$ ros2 pkg create test
going to create a new package
package name: test
...略
```

## 3-3.「launch」サブコマンド

複数のROSノードの起動をまとめて行いたいときなどにはlaunchファイルを書くことができます。ROS1ではXMLによる記述しかできませんでしたが、
ファイルを読み込んで起動条件を変えたり、[起動中のイベントを判断して反応するlaunchファイル](https://github.com/ros2/launch/blob/master/launch/examples/launch_counters.py)なども実装可能です。

試しにlaunchファイルを書いてみましょう。

```python test.launch.py
import launch
import launch_ros.actions


def generate_launch_description():
    greeter = launch_ros.actions.Node(
        package='greeter',
        node_executable='greeter',
        output='screen'
        )
    displayer = launch_ros.actions.Node(
        package='displayer',
        node_executable='displayer',
        output='screen'
        )

    return launch.LaunchDescription([
        greeter,
        displayer,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=displayer,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )),
            ])
```

使用するときは以下のコマンドで使います。

```shell
. install/local_setup.bash
ros2 launch test.launch.py
```

[4.作ってみよう に進む](4_Turtle.md)
