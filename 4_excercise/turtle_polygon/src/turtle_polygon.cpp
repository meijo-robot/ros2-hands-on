// Copyright 2019 Geoffrey Biggs
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <memory>
#include <turtlesim/action/rotate_absolute.hpp>
#include "geometry_msgs/msg/twist.hpp"


using namespace std::chrono_literals;
using TurtleAction = turtlesim::action::RotateAbsolute;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("turtle_polygon");

  // アクションクライントを作成する
  auto client = rclcpp_action::create_client<TurtleAction>(node, "turtle1/rotate_absolute");
  // パブリッシャーを作成する
  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

  // アクションが提供されるまで待つ
  while (!client->wait_for_action_server(1s)) {
    // シャットダウンされたかどうか確認する
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for action server.");
      rclcpp::shutdown();
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for action server...");
  }

  //
  // ここにコードを記述して亀に四角形を描かせよう。
  //

  rclcpp::shutdown();
  return 0;
}
