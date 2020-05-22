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
  float angle = 0;
  auto vel = geometry_msgs::msg::Twist();
  vel.linear.x=3.0;
  
  for( int i = 0; i < 4 ; i++)
  {
    // アクションGoalを作成する
    auto goal = TurtleAction::Goal();
    goal.theta = angle ;

    RCLCPP_INFO(node->get_logger(), "Sending goal");
    using std::placeholders::_1;
    using std::placeholders::_2;
    auto send_goal_options = rclcpp_action::Client<TurtleAction>::SendGoalOptions();
    // send_goal_options.feedback_callback = std::bind(&feedback_callback, node, _1, _2);
    auto goal_handle_future = client->async_send_goal(goal, send_goal_options);

    // Goalがサーバーでアクセプトされるまで待つ
    if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
        rclcpp::executor::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Send goal call failed");
      rclcpp::shutdown();
      return 1;
    }

    rclcpp_action::ClientGoalHandle<TurtleAction>::SharedPtr goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
      rclcpp::shutdown();
      return 1;
    }

    // サーバーでアクションの実行が終わるまで待つ
    RCLCPP_INFO(node->get_logger(), "Waiting for result");
    auto result_future = client->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(node, result_future) !=
        rclcpp::executor::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Failed to get action result");
      rclcpp::shutdown();
      return 1;
    }

    
    RCLCPP_INFO(node->get_logger(), "直進！！");
    publisher->publish(vel);//直進させる

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    angle += 1.57;
  }

  rclcpp::shutdown();
  return 0;
}
