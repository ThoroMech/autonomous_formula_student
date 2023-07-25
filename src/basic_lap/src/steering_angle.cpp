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
    publisher2_ = this->create_publisher<geometry_msgs::msg::Point>("desired_loc", 10);

    RCLCPP_INFO(this->get_logger(), "Steering Angle Algorithm Initialised");
  }
private:
  const float gain = 1.5;
  void topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    float steering_angle = 0.0;
    steering_angle = -gain * desired_angle(msg);

    std_msgs::msg::Float32 steering_angle_msg;
    steering_angle_msg.data = steering_angle;
    publisher_->publish(steering_angle_msg);

    //RCLCPP_INFO(this->get_logger(), "Steering angle message: %f", steering_angle);
  }

  double desired_angle(const geometry_msgs::msg::PoseArray::SharedPtr cones)
  {
    const double coneIgnoreDist = 1.0; // Ignore desired location if within this distance of a cone

    std::pair<double, double> averages = get_closest_cone_positions(cones);
    double avgX = averages.first;
    double avgY = averages.second;

    // Check if the average location is within the ignore distance of any cone
    for (const auto& cone : cones->poses)
    {
        double dist = std::hypot(cone.position.x - avgX, cone.position.y - avgY);
        if (dist < coneIgnoreDist)
        {
        return std::atan2(avgY, avgX); // Return the previous desired angle
        }
    }

    //publish the average x and y coordinate
    geometry_msgs::msg::Point desired_loc;
    desired_loc.x = avgX;
    desired_loc.y = avgY;
    publisher2_->publish(desired_loc);

    // Calculate the desired steering angle
    return std::atan2(avgY, avgX);
  }

  std::pair<double, double> get_closest_cone_positions(const geometry_msgs::msg::PoseArray::SharedPtr& cones)
  {
      std::vector<double> distances;
      for (const auto& cone : cones->poses) {
          double dx = cone.position.x;
          double dy = cone.position.y;
          double distance = std::sqrt(dx*dx + dy*dy);
          distances.push_back(distance);
      }

      std::sort(distances.begin(), distances.end());

      double total_x = 0.0;
      double total_y = 0.0;
      int num_cones = std::min(4, static_cast<int>(cones->poses.size()));
      for (int i = 0; i < num_cones; i++) {
          double closest_distance = distances[i];
          for (const auto& cone : cones->poses) {
              double dx = cone.position.x;
              double dy = cone.position.y;
              double distance = std::sqrt(dx*dx + dy*dy);
              if (std::abs(distance - closest_distance) < 1e-6) {
                  total_x += cone.position.x;
                  total_y += cone.position.y;
                  break;
              }
          }
      }

      double average_x = total_x / num_cones;
      double average_y = total_y / num_cones;
      return std::make_pair(average_x, average_y);
  }

  

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher2_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ConeLocationSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}