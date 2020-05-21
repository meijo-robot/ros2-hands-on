# 2-4. アクション

## 2-4-1. 定義方法

アクションは何かのリクエストを受け取って値を返す場合のメッセージタイプです。
基本的にはサービスと同じですが、実行中の値を何か返すことができるという点が異なります。

独自メッセージと同様に、.actionという拡張子のファイルにクライアントからもらう値と返す値を記述して定義します

```s ProcessGreeting.action
# Goal
string name
---
# Result
string greeting
---
# Feedback　逐次返す値
uint8 percent_complete
```

## 2-4-2 使い方

action server はリクエストを受け取るコールバック関数をそれぞれ定義します。
ここでは抜粋のみ。詳細はソースコードを見てください

```c++ greeting_processor_component.cpp
 server_ = rclcpp_action::create_server<ProcessGreeting>(
    this->shared_from_this(),
    "process_greeting",
    std::bind(&GreetingProcessor::handle_goal, this, _1, _2),
    std::bind(&GreetingProcessor::handle_cancel, this, _1),
    std::bind(&GreetingProcessor::handle_accepted, this, _1));
```

クライアント側も少し複雑なので、ソースコードを参照ください

## 2-2-2.ビルド

ビルドする前に2_3_service_message/COLCON_IGNOREをリネームもしくは削除してください。
以下のコマンドでビルドしてみましょう。

``` shell
cd ~/ros2_basics
colcon build --merge-install
```

## 2-2-4.実行

2つの端末でそれぞれ以下のコマンドを入力して実行してみましょう
[端末 A]

```shell
. ~/ros2_basics/install/local_setup.bash
ros2 run greeting_processor greeting_processor
```

[端末 B]

```shell
. ~/ros2_basics/install/local_setup.bash
ros2 run greet_me greet_me
ros2 run greet_me_with_cancel greet_me
```

## 2-4-4.メッセージの確認

ros2 action のサブコマンドで現在動いているアクションサービスの一覧を確認することができます。

```shell
. ~/ros2_basics/install/local_setup.bash
ros2 action list
```

[3. その他のツール　へ進む](3_ROS2_TOOLS.md)
