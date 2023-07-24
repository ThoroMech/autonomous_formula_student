#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "fs_msgs/msg/control_command.hpp"

class MyNode : public rclcpp::Node {
public:
  MyNode() : Node("my_node") 
  {
    steering_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "steering_angle", 10, std::bind(&MyNode::steering_callback, this, std::placeholders::_1));
    throttle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "throttle_pos", 10, std::bind(&MyNode::throttle_callback, this, std::placeholders::_1));
    control_pub_ = this->create_publisher<fs_msgs::msg::ControlCommand>("control_command", 10);

    RCLCPP_INFO(this->get_logger(), "Controller Messenger Initialised");
  }
private:
  double steering_;
  double throttle_;
  void steering_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    steering_ = msg->data;
    // RCLCPP_INFO(this->get_logger(), "Steering Position recieved as %f", steering_);
    publish_control_command();
  }

  void throttle_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    throttle_ = msg->data;
    // RCLCPP_INFO(this->get_logger(), "Throttle Position recieved as %f", throttle_);
    publish_control_command();
  }

  void publish_control_command()
  {
    fs_msgs::msg::ControlCommand control_msg;
    control_msg.header.stamp = this->now();
    control_msg.throttle = throttle_;
    control_msg.steering = steering_;
    control_msg.brake = 0.0;  // Set brake position to 0

    control_pub_->publish(control_msg);
  }
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr throttle_sub_;
  rclcpp::Publisher<fs_msgs::msg::ControlCommand>::SharedPtr control_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNode>());
  rclcpp::shutdown();
  return 0;
}