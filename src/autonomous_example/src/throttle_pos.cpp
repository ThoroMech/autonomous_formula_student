#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <cmath>
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class GssSubscriber : public rclcpp::Node {
public:
  GssSubscriber() : Node("gss_subscriber") 
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/gss", 10, std::bind(&GssSubscriber::callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("throttle_pos", 10);

    RCLCPP_INFO(this->get_logger(), "Throttle Position Algorithm Initialised");
  }
private:
  void callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) 
  {
    const int target_speed = 8.5;
    const float max_throttle = 1;
    
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    double v_mag = std::sqrt(vx * vx + vy * vy);

    float throttle_pos = max_throttle * std::max(1.0-v_mag/target_speed, 0.0);

    std_msgs::msg::Float32 throttle_pos_msg;
        throttle_pos_msg.data = throttle_pos;
        publisher_->publish(throttle_pos_msg);

    // RCLCPP_INFO(this->get_logger(), "Throttle Postition message %f", throttle_pos);
  }
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GssSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
