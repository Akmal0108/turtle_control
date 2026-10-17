#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class CircleDriver : public rclcpp::Node
{
public:
  CircleDriver()
  : Node("circle_driver")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    timer_ = this->create_wall_timer(
      100ms, std::bind(&CircleDriver::timer_callback, this));
  }

private:
  void timer_callback()
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 2.0;   // maju 2 unit/detik
    msg.angular.z = 1.0;  // putar kiri 1 rad/detik, radius = 2.0 / 1.0 = 2 unit
    publisher_->publish(msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CircleDriver>());
  rclcpp::shutdown();
  return 0;
}