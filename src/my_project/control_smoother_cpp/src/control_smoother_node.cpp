#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class ControlSmoother : public rclcpp::Node
{
public:
  ControlSmoother()
  : Node("control_smoother"),
    alpha_(declare_parameter<double>("alpha", 0.2)),     // 필터 강도
    rate_hz_(declare_parameter<double>("rate_hz", 100.0))
  {
    raw_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/raw_cmd",
      10,
      std::bind(&ControlSmoother::rawCmdCallback, this, std::placeholders::_1));

    smooth_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/smooth_cmd",
      10);

    current_cmd_ = geometry_msgs::msg::Twist();
    target_cmd_  = geometry_msgs::msg::Twist();

    auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ControlSmoother::timerCallback, this));
  }

private:
  void rawCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // 새로운 타겟 명령만 저장 (바로 퍼블리시하지 않음)
    target_cmd_ = *msg;
  }

  void timerCallback()
  {
    // 1차 필터: current += alpha * (target - current)
    current_cmd_.linear.x  += alpha_ * (target_cmd_.linear.x  - current_cmd_.linear.x);
    current_cmd_.linear.y  += alpha_ * (target_cmd_.linear.y  - current_cmd_.linear.y);
    current_cmd_.linear.z  += alpha_ * (target_cmd_.linear.z  - current_cmd_.linear.z);

    current_cmd_.angular.x += alpha_ * (target_cmd_.angular.x - current_cmd_.angular.x);
    current_cmd_.angular.y += alpha_ * (target_cmd_.angular.y - current_cmd_.angular.y);
    current_cmd_.angular.z += alpha_ * (target_cmd_.angular.z - current_cmd_.angular.z);

    smooth_cmd_pub_->publish(current_cmd_);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr raw_cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    smooth_cmd_pub_;
  rclcpp::TimerBase::SharedPtr                               timer_;

  geometry_msgs::msg::Twist current_cmd_;
  geometry_msgs::msg::Twist target_cmd_;

  double alpha_;
  double rate_hz_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControlSmoother>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
