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

action server は3つのコールバック関数を引数に必要とします。

- handle_goal : goalを受け取った場合の動作を記述する関数。goalの良不判定等
- handle_cancel : 実行中にcancelを受け取った際の動作を記述する関数
- handle_accepted : handle_goalが受付られた場合に実行される関数

それぞれ別々に定義が必要です。

ここではaction_serverの作成部分のみ抜粋。詳細はソースコードを見てください

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

```shell
cd ~/ros2_basics
colcon build --merge-install
```

## 2-2-4.実行

2つの端末でそれぞれ以下のコマンドを入力して実行してみましょう

[端末 A]

```shell
$ . ~/ros2_basics/install/local_setup.bash
$ ros2 run greeting_processor greeting_processor

# 以下は端末Bを実行後に表示されます
[INFO] [server]: Got request to process greeting from human 'Bill'
[INFO] [server]: Beginning execution of goal
[INFO] [server]: Executing goal for human 'Bill'
[INFO] [server]: 0 complete
[INFO] [server]: 10 complete
[INFO] [server]: 20 complete
[INFO] [server]: 30 complete
[INFO] [server]: 40 complete
[INFO] [server]: 50 complete
[INFO] [server]: 60 complete
[INFO] [server]: 70 complete
[INFO] [server]: 80 complete
[INFO] [server]: 90 complete
[INFO] [server]: 100 complete
[INFO] [server]: Goal succeeded
```

[端末 B]

コマンドを実行すると以下のように表示されます。

```shell
$ . ~/ros2_basics/install/local_setup.bash
$ ros2 run greet_me greet_me
[INFO] [greet_me]: Sending goal: 'Bill'
[INFO] [greet_me]: Waiting for result
[INFO] [greet_me]: Greeting creation 0% complete
[INFO] [greet_me]: Greeting creation 10% complete
[INFO] [greet_me]: Greeting creation 20% complete
[INFO] [greet_me]: Greeting creation 30% complete
[INFO] [greet_me]: Greeting creation 40% complete
[INFO] [greet_me]: Greeting creation 50% complete
[INFO] [greet_me]: Greeting creation 60% complete
[INFO] [greet_me]: Greeting creation 70% complete
[INFO] [greet_me]: Greeting creation 80% complete
[INFO] [greet_me]: Greeting creation 90% complete
[INFO] [greet_me]: Greeting creation 100% complete
[INFO] [greet_me]: Received action result: 'Hello, Bill'
```

アクションの実行中に途中でキャンセルするプログラムも走らせてみましょう。
[端末B]で下記のコマンドを実行してください。

```shell
$ ros2 run greet_me_with_cancel greet_me
[INFO] [greet_me]: Sending goal: 'Bill'
[INFO] [greet_me]: Waiting for result
[INFO] [greet_me]: Greeting creation 10% complete
[INFO] [greet_me]: Greeting creation 20% complete
[INFO] [greet_me]: Greeting creation 30% complete
[INFO] [greet_me]: Greeting creation 40% complete
[INFO] [greet_me]: Greeting creation 50% complete
[INFO] [greet_me]: Cancelling goal
[INFO] [greet_me]: Waiting for result again
[ERROR] [greet_me]: Goal was cancelled
```

## 2-4-4.メッセージの確認

ros2 action のサブコマンドで現在動いているアクションの一覧を確認することができます。

```shell
$ . ~/ros2_basics/install/local_setup.bash
$ ros2 action list -t
/process_greeting [greeting_actions/action/ProcessGreeting]
```

アクションの型はshowサブコマンドで確認可能です

```shell
$ ros2 action show greeting_actions/action/ProcessGreeting
# Goal
string name
---
# Result
string greeting
---
# Feedback
uint8 percent_complete
```

サービスと同様に、actionもコマンドで動作確認ができます。上の[端末A]が動いている状態で、以下のコマンドを実行すると、アクションを実行[することができます。

```shell
$ ros2 action send_goal -f  /process_greeting greeting_actions/action/ProcessGreeting "{name: \"test\"}"
Waiting for an action server to become available...
Sending goal:
     name: test

Goal accepted with ID: a15eae182b734995bfb80bf4f742fd58

Feedback:
    percent_complete: 0

Feedback:
    percent_complete: 10

Feedback:
    percent_complete: 20

Feedback:
    percent_complete: 30

Feedback:
    percent_complete: 40

Feedback:
    percent_complete: 50

Feedback:
    percent_complete: 60

Feedback:
    percent_complete: 70

Feedback:
    percent_complete: 80

Feedback:
    percent_complete: 90

Feedback:
    percent_complete: 100

Result:
    greeting: Hello, test

Goal finished with status: SUCCEEDED
```

確認できたら各ターミナルで[Ctrl + c]を押してプログラムを停止してください。

[3. その他のツール　へ進む](3_ROS2_TOOLS.md)
