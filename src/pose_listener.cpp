#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

using std::placeholders::_1;

class PoseListener : public rclcpp::Node
{
public:
  PoseListener()
  : Node("pose_listener")
  {
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10, std::bind(&PoseListener::pose_callback, this, _1));
  }

private:
  void pose_callback(const turtlesim::msg::Pose & msg)
  {
    // Turtlesim mem-publish pose sekitar 60 kali/detik, jadi dibatasi 1 log per detik.
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "x=%.2f  y=%.2f  theta=%.2f rad", msg.x, msg.y, msg.theta);
  }

  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseListener>());
  rclcpp::shutdown();
  return 0;
}