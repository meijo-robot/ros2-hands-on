# 2-1.ROS2 APIの基本

この章で行うこと

1. リポジトリの準備
2. 基本的なROS2スタイルの実装(送信　受信)
3. 独自のメッセージ型の定義と利用方法
4. サービスの利用
5. アクションの利用
6. ノードのコンポジション(ROS1 node-let)

## 2-1-1.リポジトリの準備

まずは、新しく端末を開いて、ワークスペースを作成しましょう

```shell
mkdir -p ~/ros2_basics/src
```

Gitからソースをダウンロードしましょう

```shell
cd ~/ros2_basics/src
git clone https://github.com/meijo-robot/ros2-hands-on.git
cd ros2-hands-on
```

## 2-1-2. 基本的なROS2スタイルの実装

まずはオーソドックスなプログラムの実装方法を見ていきましょう

### ビルドの準備

まず、2_1_sending_message/COLCON_IGNOREファイルをリネームするか削除してください。
(COLCON_IGNOREファイルが設置されているディレクトリはcolconから無視されます)

ROSのノードを実装する際に必要なのがpackage.xml とCMakeLists.txtです。
それぞれ以下の役目があります。

- package.xml : プログラムに必要な依存関係を記述
- CMakeLists : プログラムのビルドの手順などを記述

最初は慣れないと思うので、一通り目を通すだけにしておきましょう。

### ソースコード

```cpp greeter_component.cpp
#include "greeter_ros2_style/greeter_component.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <class_loader/register_macro.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

namespace greeter_ros2_style
{

Greeter::Greeter(const rclcpp::NodeOptions & options)
: Node("greeter", options)
{
  pub_ = create_publisher<std_msgs::msg::String>("greeting", 10);

  // ノードの振舞いを定期的に実行するためにタイマーを利用する
  timer_ = create_wall_timer(1s, std::bind(&Greeter::broadcast_greeting, this));
}

// ノードの振舞い
// タイマーイベント毎に実行される
void Greeter::broadcast_greeting()
{
  auto greeting = std::make_shared<std_msgs::msg::String>();
  greeting->data = "hello world";
  // ロガーに出力する
  RCLCPP_INFO(this->get_logger(), "Publishing greeting '%s'", greeting->data.c_str());
  pub_->publish(*greeting);
}

} // namespace greeter_ros2_style

#include "rclcpp_components/register_node_macro.hpp"

// 動的にコンポーネントノードをロードできるために登録する
RCLCPP_COMPONENTS_REGISTER_NODE(greeter_ros2_style::Greeter)

```

プログラム本体です。クラスとして定義したノードをこのプログラムから呼び出しています

```cpp greeter.cpp
#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "greeter_ros2_style/greeter_component.hpp"

int main(int argc, char *argv[]) {
  // ROSをイニシャライズする
  rclcpp::init(argc, argv);
  // コンポーネントノードのインスタンスを作成する
  rclcpp::NodeOptions options;
  auto greeter = std::make_shared<greeter_ros2_style::Greeter>(options);
  // コンポーネントノードを実行する
  rclcpp::spin(greeter);
  // Shut down ROS
  rclcpp::shutdown();
  return 0;
}

```

### ビルド & 実行

プログラムのビルドは以下のコマンドで行います。

``` shell
$ cd ~/ros2_basics/
$ colcon build
(略)
Finished <<< displayer_basic_version [0.35s]
Finished <<< greeting [0.35s]

Summary: 2 packages finished [0.49s]
```

sourceコマンドは"."で代替が可能です。こちらのほうが短く記述できるのでこれ以降、この方法を使います。

``` shell
$ . install/local_setup.bash
# 実行後は何も表示されません
```

```shell
$ ros2 run greeting greeter
[INFO] [greeter]: Publishing greeting 'hello world'
[INFO] [greeter]: Publishing greeting 'hello world'
[INFO] [greeter]: Publishing greeting 'hello world'
```

greeterを起動中に下記のコマンドで起動中のノードやトピックを確認できます。
コマンドを実行して確認してみましょう。

```shell
$ ros2 node list
/greeter
$ ros2 topic list -t
/greeting [std_msgs/msg/String]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
```

- ``` ros2 topic list ```では-tオプションをつけることで、トピックの型を確認できます。
- ``` rost node ``` ``` rost topic ``` に-hオプションをつけることで各コマンドのヘルプが表示されます。コマンドの詳細がわからない場合、これらで確認してください。

確認できたら各ターミナルで[Ctrl + c]を押してプログラムを停止してください。

## 2-1-2.受信ノードの準備

Displayerで重要な箇所はdisplayer_component.cppです。
~/ros2_basics/src/ros2-hands-on/receiving_messages/displayer/src/displayer_component.cppを開いて確認しましょう。

### displayer_component.cppのソースコード

```cpp displayer_component.cpp

Displayer::Displayer(const rclcpp::NodeOptions & options)
: Node("displayer", options)
{
　//29行目
　// メッセージを受信した際のトピックとハンドラ関数を関連付ける
  sub_ = this->create_subscription<std_msgs::msg::String>(
    "greeting", 10, std::bind(&Displayer::display_greeting, this, _1));
}

//ハンドラ関数
void Displayer::display_greeting(const std_msgs::msg::String::SharedPtr msg)
{
  //35行目
  // ロガーに受け取ったメッセージを出力する
  RCLCPP_INFO(this->get_logger(), "Received greeting '%s'", msg->data.c_str());
}
```

### ビルド・実行

それではdisplayerのビルドを行っていきましょう。新しいターミナルを開いて以下のコマンドでビルドできます。

```sh
$ cd ~/ros2_basics/
$ colcon build
Starting >>> displayer_basic_version
Starting >>> greeting
Finished <<< displayer_basic_version [0.35s]
Finished <<< greeting [0.35s]

Summary: 2 packages finished [0.49s]
```

2つのターミナルにで以下のコマンドを実行してください。
端末Bを実行後に端末Aに'[INFO] [displayer]: Received greeting 'hello world''と表示されれば成功です。

[端末A]

``` shell
$ . install/local_setup.bash
$ ros2 run displayer_basic_version displayer
#端末B実行後に表示
[INFO] [displayer]: Received greeting 'hello world'
[INFO] [displayer]: Received greeting 'hello world'
[INFO] [displayer]: Received greeting 'hello world'
```

[端末B]

```shell
$ . install/local_setup.bash
$ ros2 run greeting greeter
[INFO] [greeter]: Publishing greeting 'hello world'
[INFO] [greeter]: Publishing greeting 'hello world'
[INFO] [greeter]: Publishing greeting 'hello world'
```

確認できたら各ターミナルで[Ctrl + c]を押してプログラムを停止してください。

[2-2.メッセージ型の定義へ進む](2_2_ROS2_msg.md)
