# 2-2.ROS2 APIの基本 2

## 2-2-1.メッセージ型の定義

メッセージ型の定義の際に行う操作は以下の通りです。

1. msgファイルとプロジェクトの作成
2. package.xmlの更新
3. CMakeLists.txtの更新
4. プログラム内で使用する

今回のディレクトリには上の操作を適用済みのファイルを用意しているので、特に操作は必要ありません。

新しい独自のメッセージ型を定義するにはmsgファイルを作成します。~/ros2_basics/src/ros2-hands-on/2_2_custom_messge/msgディレクトリ下にGreeting.msgという名前のファイルがあります。
中身を確認してみましょう。

### Greeting.msgのファイルの中身

``` Greeting.msg
string hello_text
string world_name
int16 count
```

msgファイルでの定義は

```text
型 フィールド名
```

で定義されます。次章、次次章のservice,actionでも同じ記述をします。

### package.xml

2_1と比較して変更を加えている箇所は以下の通りです

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

### CMakeLists.txt

2_1と比較して変更を加えている箇所は以下の通りです

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

ビルドする前に2_2_custom_message/COLCON_IGNOREをリネームもしくは削除してください
以下のコマンドで、ビルドしてみましょう

``` shell
cd ~/ros2_basics
colcon build
```

## 2-2-3.実行

2つの端末でそれぞれ以下のコマンドを入力して実行してみましょう

### [端末 A]

```shell
. ~/ros2_basics/install/local_setup.bash
ros2 run displayer_custom_msg diplayer_custom
```

### [端末 B]

```shell
$ . ~/ros2_basics/install/local_setup.bash
$ ros2 run greeter_custom_msg greeter_custom
[INFO] [greeter]: Publishing greeting 'hello world 0'
[INFO] [greeter]: Publishing greeting 'hello world 1'
[INFO] [greeter]: Publishing greeting 'hello world 2'
[INFO] [greeter]: Publishing greeting 'hello world 3'
[INFO] [greeter]: Publishing greeting 'hello world 4'
......
```

[端末A]に以下のように表示されれば成功です

```shell
[INFO] [displayer]: Received greeting 'hello world 0'
[INFO] [displayer]: Received greeting 'hello world 1'
[INFO] [displayer]: Received greeting 'hello world 2'
[INFO] [displayer]: Received greeting 'hello world 3'
[INFO] [displayer]: Received greeting 'hello world 4'
[INFO] [displayer]: Received greeting 'hello world 5'
```

## 2-2-4.メッセージの確認

ros2 topic のサブコマンドでtopicの一覧を見ることができます。(-t オプションをつけることでトピック名とその型をみることができます)

```shell
$ . ~/ros2_basics/install/local_setup.bash
$ ros2 topic list -t
/greeting [greeting_msg/msg/Greeting]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
```

また、独自に定義したメッセージ型の一覧を見たい場合にはmsgサブコマンドを使います

```shell
$ . ~/ros2_basics/install/local_setup.bash
_
$ ros2 msg list | grep greet
greeting_msg/msg/Greeting
```

[2-3サービス に進む](2_3_ROS2_srv.md)
