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

まず、ROSのノードを実装する際に必要なのがpackage.xml とCMakeLists.txtです。
それぞれ以下の役目があります。

- package.xml : プログラムに必要な依存関係を記述
- CMakeLists : プログラムのビルドの手順などを記述

先程のコマンドで作ったパッケージにすでに上記のファイルができています。
それぞれのファイルを以下のように編集していきましょう。

#### package.xml

```xml package.xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format2.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>greeter_ros2_style</name>
  <version>1.2.0</version>
  <description>講習会用のサンプルソース：ROS2形式のメッセージ送信ノード</description>
  <maintainer email="git@killbots.net">Geoffrey Biggs</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <build_depend>rclcpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>rclcpp_components</build_depend>

  <exec_depend>rclcpp</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>rclcpp_components</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>

```

#### CMakeLists.txt

```cmake CMakeList.txt
cmake_minimum_required(VERSION 3.5)
# パッケージ名
project(greeter_ros2_style)

# C++14を利用する
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# すべてのワーニングを表示する
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依存するパッケージを探す
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(include)

# コンポーネントノードの共有ライブラリをコンパイルする
add_library(greeter_component SHARED src/greeter_component.cpp)
# コンポーネントノードのヘッダーファイルのマクロを設定する
target_compile_definitions(greeter_component PRIVATE "GREETER_BUILDING_DLL")
ament_target_dependencies(greeter_component
  rclcpp
  std_msgs
  rclcpp_components
  )
# コンポーネントノードをamentのリソースインデクスに登録する
rclcpp_components_register_nodes(greeter_component "greeter_ros2_style::Greeter")

# スタンドアローンノードの実行ファイルをコンパイルする
add_executable(greeter src/greeter.cpp)
# 共有ライブラリに実行ファイルをリンクする
target_link_libraries(greeter greeter_component)
# 実行ファイルのコンパイルターゲットに依存パッケージの情報を追加する
ament_target_dependencies(greeter rclcpp std_msgs)

# コンポーネントノードのヘッダーファイルをインストールする
install(DIRECTORY
  include/greeter_ros2_style
  DESTINATION include
  )

# コンポーネントノードの共有ライブラリをインストールする
install(TARGETS
  greeter_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

# スタンドアローンの実行ファイルをインストールする
install(TARGETS
  greeter
  DESTINATION lib/${PROJECT_NAME}
  )

# amentのリソースインデクスにパッケージを登録する
ament_package()

```

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

最後が、プログラム本体です。クラスとして定義したノードをこのプログラムから呼び出しています

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

``` shell
cd ~/ros2_basics/
colcon build --packages-select greeter_ros2_style
```

``` shell
source install/local_setup.bash
ros2 run greeter_ros2_style greeter
```

## 2-1-2.受信ノードの準備

すでに受信側のソースコードは準備してありますが、不足している部分があります。

~/ros2_basics/src/ros2-hands-on/receiving_messages/displayer/src/displayer_component.cppを開いて修正していきましょう

### displayer.cppのソースコード

```cpp displayer.cpp

Displayer::Displayer(const rclcpp::NodeOptions & options)
: Node("displayer", options)
{
　//29行目
　// メッセージを受信した際のトピックとハンドラ関数を関連付ける
  sub_ = this->create_subscription<std_msgs::msg::String>(
    "greeting", 10, std::bind(&Displayer::display_greeting, this, _1));
}

//実際のアクション
void Displayer::display_greeting(const std_msgs::msg::String::SharedPtr msg)
{
  //35行目
  // ロガーに受け取ったメッセージを出力する
  RCLCPP_INFO(this->get_logger(), "Received greeting '%s'", msg->data.c_str());
}
```

### ビルド・実行

それではdisplayerのビルドを行っていきましょう.以下のコマンドでビルドできます。

```sh
cd ~/ros2_basics/
colcon build --packages-select displayer_basic_version
```

実行は以下のコマンドです。

``` shell
source install/local_setup.bash
ros2 run greeter_ros2_style greeter
```

[2-2.メッセージ型の定義へ進む](2_2_ROS2_msg.md)
