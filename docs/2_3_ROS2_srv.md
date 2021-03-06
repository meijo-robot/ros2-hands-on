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

ビルドする前に2_3_service_message/COLCON_IGNOREをリネームもしくは削除してください。
以下のコマンドでビルドしてみましょう。

``` shell
$ cd ~/ros2_basics
$ colcon build
(略)
```

## 2-3-3.実行

2つの端末でそれぞれ以下のコマンドを入力して実行してみましょう。

[端末 A]

```shell
$. ~/ros2_basics/install/local_setup.bash
$ ros2 run greeting_server greeting_server
[INFO] [greeting_server]: Responding to greeting request with 'Hello, Bob'
```

[端末 B]

```shell
$ . ~/ros2_basics/install/local_setup.bash
$ ros2 run greeting_client greeting_client
[INFO] [greeting_client]: Received greeting: 'Hello, Bob'
```

## 2-3-4.サービスの確認

ros2 service listのサブコマンドで現在動いているサービスの一覧を確認することができます。
```-t```オプションをつけることで、サービスの型を確認することができます

```shell
$ . ~/ros2_basics/install/local_setup.bash
$ ros2 service list -t
/greeting_server/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/greeting_server/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/greeting_server/get_parameters [rcl_interfaces/srv/GetParameters]
/greeting_server/list_parameters [rcl_interfaces/srv/ListParameters]
/greeting_server/set_parameters [rcl_interfaces/srv/SetParameters]
/greeting_server/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/request_greeting [request_greeting_service/srv/RequestGreeting]
```

定義されているメッセージ型の一覧を見たい場合にはsrvサブコマンドを使います。
下記ではgrepを使って、今回作成した型のみを表示させています。

```shell
$ . ~/ros2_basics/install/local_setup.bash
$ ros2 srv list | grep greet
request_greeting_service/srv/RequestGreeting
```

```ros2 service call```を使用し、サービスにリクエストをコマンドで送ることも可能です。

```ros2 service call [サービス名] [サービスの型] "サービスに渡す値"```

で実行します。以下は実行例です。

```shell
$ . ~/ros2_basics/install/local_setup.bash
$ ros2 service call /request_greeting request_greeting_service/srv/RequestGreeting "{name: \"test\"}"
waiting for service to become available...
requester: making request: request_greeting_service.srv.RequestGreeting_Request(name='test')

response:
request_greeting_service.srv.RequestGreeting_Response(greeting='Hello, test')
```

確認できたら各ターミナルで[Ctrl + c]を押してプログラムを停止してください。

[2-4. アクション　へ進む](2_4_ROS2_action.md)
