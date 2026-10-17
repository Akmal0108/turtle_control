#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class SquareDriver : public rclcpp::Node
{
public:
  SquareDriver()
  : Node("square_driver"),
    pose_received_(false),
    initialized_(false),
    phase_(Phase::MOVE),
    side_count_(0),
    start_x_(0.0),
    start_y_(0.0),
    target_theta_(0.0)
  {
    cmd_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>(
        "/turtle1/cmd_vel", 10);

    pose_sub_ =
      this->create_subscription<turtlesim::msg::Pose>(
        "/turtle1/pose", 10,
        std::bind(&SquareDriver::pose_callback, this, _1));

    timer_ = this->create_wall_timer(
      50ms, std::bind(&SquareDriver::timer_callback, this));

    RCLCPP_INFO(
      this->get_logger(), "SquareDriver menunggu pose...");
  }

private:
  enum class Phase {
    MOVE,
    TURN,
    DONE
  };

  static constexpr double PI = 3.14159265358979323846;
  static constexpr double SIDE_LENGTH = 2.0;
  static constexpr double DIST_TOL = 0.05;
  static constexpr double ANGLE_TOL = 0.03;

  double x_ = 0.0;
  double y_ = 0.0;
  double theta_ = 0.0;

  bool pose_received_;
  bool initialized_;
  Phase phase_;
  int side_count_;

  double start_x_;
  double start_y_;
  double target_theta_;

  rclcpp::Publisher<
    geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  rclcpp::Subscription<
    turtlesim::msg::Pose>::SharedPtr pose_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  void pose_callback(const turtlesim::msg::Pose & pose)
  {
    x_ = pose.x;
    y_ = pose.y;
    theta_ = pose.theta;
    pose_received_ = true;
  }

  double angle_error(double target, double current)
  {
    const double error = target - current;
    return std::atan2(std::sin(error), std::cos(error));
  }

  void timer_callback()
  {
    geometry_msgs::msg::Twist cmd;

    // Tunggu sampai pose pertama diterima.
    if (!pose_received_) {
      return;
    }

    // Simpan posisi dan arah awal.
    if (!initialized_) {
      start_x_ = x_;
      start_y_ = y_;
      target_theta_ = theta_;
      initialized_ = true;

      RCLCPP_INFO(
        this->get_logger(),
        "Mulai menggambar persegi.");
    }

    // Jika selesai, terus kirim perintah berhenti.
    if (phase_ == Phase::DONE) {
      cmd_pub_->publish(cmd);
      return;
    }

    const double error_theta =
      angle_error(target_theta_, theta_);

    if (phase_ == Phase::MOVE) {
      const double dx = x_ - start_x_;
      const double dy = y_ - start_y_;
      const double distance = std::hypot(dx, dy);
      const double remaining = SIDE_LENGTH - distance;

      if (remaining <= DIST_TOL) {
        // Sisi selesai: mulai berbelok 90 derajat.
        phase_ = Phase::TURN;
        target_theta_ =
          std::atan2(
            std::sin(target_theta_ + PI / 2.0),
            std::cos(target_theta_ + PI / 2.0));

        RCLCPP_INFO(
          this->get_logger(),
          "Sisi %d selesai, mulai berbelok.",
          side_count_ + 1);
      } else {
        // Kontrol arah saat bergerak lurus.
        cmd.linear.x = std::clamp(
          1.0 * remaining, 0.0, 1.0);

        cmd.angular.z = std::clamp(
          2.0 * error_theta, -1.0, 1.0);
      }
    }

    if (phase_ == Phase::TURN) {
      const double turn_error =
        angle_error(target_theta_, theta_);

      if (std::fabs(turn_error) < ANGLE_TOL) {
        // Sudut sudah cukup dekat dengan 90 derajat.
        side_count_++;

        if (side_count_ >= 4) {
          phase_ = Phase::DONE;

          RCLCPP_INFO(
            this->get_logger(),
            "Persegi selesai. Berhenti.");
        } else {
          start_x_ = x_;
          start_y_ = y_;
          phase_ = Phase::MOVE;

          RCLCPP_INFO(
            this->get_logger(),
            "Mulai sisi %d.",
            side_count_ + 1);
        }
      } else {
        cmd.angular.z = std::clamp(
          2.0 * turn_error, -1.0, 1.0);
      }
    }

    cmd_pub_->publish(cmd);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SquareDriver>());
  rclcpp::shutdown();
  return 0;
}