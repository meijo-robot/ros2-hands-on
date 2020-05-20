# 2-3. サービス

## 2-3-1. 定義方法

サービスは何らかのリクエストを受け取って値を返す場合のメッセージタイプです

独自メッセージと同様に、.srvという拡張子のファイルにクライアントからもらう値と返す値を記述して定義します

```srv RequestGreeting.srv
string name # serverがもらう値の型 request
---
string greeting # クライアントに返す値の型 response
```

## 2-3-2 使い方

server はリクエストを受け取るコールバック関数として定義します。

```c++ greeting_server_component.cpp
void GreetingServer::send_greeting(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<RequestGreeting::Request> request,
  const std::shared_ptr<RequestGreeting::Response> response)
{
  (void)request_header;
  response->greeting = "Hello, " + request->name;
  RCLCPP_INFO(
    this->get_logger(),
    "Responding to greeting request with '%s'",
    response->greeting.c_str());
}
```

クライアント側は以下のようにして使います

```c++ greeting_client.cpp

　while (!client->wait_for_service(1s)) {
     /*サーバーが起動するまで待機する*/
　}
　auto request = std::make_shared<RequestGreeting::Request>();
  request->name = "Bob";
  // サーバーにリクエストを送る（非同期）
  auto result = client->async_send_request(request);
  // 結果を待つ
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), 
      "Received greeting: '%s'", result.get()->greeting.c_str());//結果の表示
  } else {
    RCLCPP_ERROR(node->get_logger(),
      "Problem while waiting for response.");
  }


```

## 2-3-2.ビルド

ビルドしてみましょう

``` shell
cd ~/ros2_basics
colcon build --merge-install
```

## 2-3-3.実行

2つの端末でそれぞれ以下のコマンドを入力して実行してみましょう
[端末 A]

```shell
. ~/ros2_basics/install/local_setup.bash
ros2 run greeting_server greeting_server
```

[端末 B]

```shell
. ~/ros2_basics/install/local_setup.bash
ros2 run greeting_client greeting_client
```

## 2-3-4.メッセージの確認

ros2 service のサブコマンドで現在動いているサービスの一覧を確認することができます。

```shell
. ~/ros2_basics/install/local_setup.bash
ros2 service list
```

また、独自に定義したメッセージ型の一覧を見たい場合にはsrvサブコマンドを使います

```shell
. ~/ros2_basics/install/local_setup.bash
ros2 srv list
```

[2-4. アクション　へ進む](2_4_ROS2_action.md)
