#include <cmath>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

using std::placeholders::_1;

class GoToGoal : public rclcpp::Node
{
public:
  GoToGoal()
  : Node("go_to_goal")
  {
    this->declare_parameter("goal_x", 8.0);
    this->declare_parameter("goal_y", 8.0);
    goal_x_ = this->get_parameter("goal_x").as_double();
    goal_y_ = this->get_parameter("goal_y").as_double();

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10, std::bind(&GoToGoal::pose_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Target: (%.2f, %.2f)", goal_x_, goal_y_);
  }

private:
  void pose_callback(const turtlesim::msg::Pose & pose)
  {
    const double dx = goal_x_ - pose.x;
    const double dy = goal_y_ - pose.y;
    const double distance = std::hypot(dx, dy);

    geometry_msgs::msg::Twist cmd;  // semua nol = berhenti
    if (distance > 0.1) {
      double error = std::atan2(dy, dx) - pose.theta;
      error = std::atan2(std::sin(error), std::cos(error));  // rapikan ke rentang [-pi, pi]

      cmd.angular.z = 4.0 * error;
      if (std::fabs(error) < 0.5) {
        cmd.linear.x = 1.5 * distance;  // maju hanya bila sudah menghadap target
      }
    }
    cmd_pub_->publish(cmd);
  }

  double goal_x_;
  double goal_y_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoToGoal>());
  rclcpp::shutdown();
  return 0;
}