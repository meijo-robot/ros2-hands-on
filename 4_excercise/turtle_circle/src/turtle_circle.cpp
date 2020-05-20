#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
using namespace std::chrono_literals;

class TurtleCircular : public rclcpp::Node
{
public:
 TurtleCircular()
  : Node("turtle_circle_node")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&TurtleCircular::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto vel = geometry_msgs::msg::Twist();
    vel.linear.x=3.0;
    vel.angular.z=3.0;
    RCLCPP_INFO(this->get_logger(), "Publishing: vel");
    publisher_->publish(vel);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleCircular>());
  rclcpp::shutdown();
  return 0;
}