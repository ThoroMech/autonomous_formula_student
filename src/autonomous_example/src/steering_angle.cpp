#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cmath>
#include <vector>

class ConeLocationSubscriber : public rclcpp::Node {
public:
  ConeLocationSubscriber() : Node("cone_location_subscriber") 
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "cone_location", 10, std::bind(&ConeLocationSubscriber::topic_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("steering_angle", 10);

    RCLCPP_INFO(this->get_logger(), "Steering Angle Algorithm Initialised");
  }
private:
  const float gain = 1.5;
  const float max_steering = 0.3;
  float steering_angle;

  void topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    //float steering_angle = 0.0;
    int counter = 0;

    for (const auto& cone : msg->poses)
    {
      counter += cone.position.y;
    }

    if (counter > 0) {steering_angle = -max_steering;}
    else {steering_angle = max_steering;}

    std_msgs::msg::Float32 steering_angle_msg;
    steering_angle_msg.data = steering_angle;
    publisher_->publish(steering_angle_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ConeLocationSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}