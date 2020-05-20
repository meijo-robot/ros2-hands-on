# 2-2.ROS2 APIの基本 2

## 2-2-1.メッセージ型の定義

メッセージ型の定義の際の操作は以下の通りです

1. msgファイルとプロジェクトの作成
2. package.xmlの更新
3. CMakeLists.txtの更新
4. プログラム内で使用する

独自でメッセージ型を定義するにはmsgファイルを作成します

~/ros2_basics/src/ros2-hands-on/2_2_custom_messge/msgディレクトリ下にGreeting.msgという名前のファイルがあります。
中身を確認してみましょう。

``` Greeting.msg
string hello_text
string world_name
int16 count
```

```xml package.xml
[略]
  <!-- ビルド時の依存関係に追加 -->
  <build_depend>greeting_msg</build_depend>
  
  <exec_depend>rclcpp</exec_depend>

  <!-- 実行時の依存関係に追加 -->
  <exec_depend>rclcpp_components</exec_depend>

  <exec_depend>greeting_msg</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
[略]
```

```cmake CMakeLists.txt
[略]
add_library(displayer_component SHARED src/displayer_component.cpp)
target_compile_definitions(displayer_component PRIVATE "DISPLAYER_BUILDING_DLL")
ament_target_dependencies(displayer_component
  rclcpp
  rclcpp_components
  greeting_msg
  )
rclcpp_components_register_nodes(displayer_component "displayer::Displayer")

add_executable(displayer src/displayer.cpp)
target_link_libraries(displayer displayer_component)

ament_target_dependencies(displayer rclcpp greeting_msg)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(greeting_msg)
[略]
```

## 2-2-2.ビルド

ビルドしてみましょう

``` shell
cd ~/ros2_basics
colcon build --merge-install
```

## 2-2-3.実行

2つの端末でそれぞれ以下のコマンドを入力して実行してみましょう
[端末 A]

```shell
. ~/ros2_basics/install/local_setup.bash
ros2 run displayer_custom_msg displayer
```

[端末 B]

```shell
. ~/ros2_basics/install/local_setup.bash
ros2 run greeter_custom_msg greeter
```

## 2-2-4.メッセージの確認

ros2 topic のサブコマンドでtopicの一覧を見ることができます。

```shell
. ~/ros2_basics/install/local_setup.bash
ros2 topic list
```

また、独自に定義したメッセージ型の一覧を見たい場合にはmsgサブコマンドを使います

```shell
. ~/ros2_basics/install/local_setup.bash
ros2 msg list
```

[2-3サービス に進む](2_3_ROS2_srv.md)
