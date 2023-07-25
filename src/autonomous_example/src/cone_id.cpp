#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "math.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"


class LidarSubscriber : public rclcpp::Node {
public:
  LidarSubscriber() : Node("lidar_subscriber") 
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar/Lidar1", 10, std::bind(&LidarSubscriber::topic_callback, this, std::placeholders::_1));
    cone_location_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("cone_location", 10);

    RCLCPP_INFO(this->get_logger(), "Cone Identification Initialised");
  }
private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Convert PointCloud2 to xyz coordinates.
    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");

    // Create a vector of tuples to store (x, y) coordinates.
    std::vector<std::tuple<float, float>> points;

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
      const float x = *iter_x;
      const float y = *iter_y;

      // Add the (x, y) coordinates to the vector.
      points.emplace_back(x, y);
    }

    // Remove points that are further away from the vehicle than (max_distance).
    const float max_distance = 7.0;
    points.erase(std::remove_if(points.begin(), 
                  points.end(),
                  [max_distance](const auto& p){
                    const float distance = sqrt (pow(std::get<0>(p), 2) +
                                                pow(std::get<1>(p), 2));
                    return distance > max_distance;
                  }), points.end()
                );

    // Group the points that are within (threshold) of each other.
    const float threshold = 0.15;  // 10cm was found to duplicate cones, 15cm was more effective
    std::vector<std::vector<std::tuple<float, float>>> cones;
    for (const auto& p1 : points) {
      bool found_cone = false;
      for (auto& cone : cones) {
        const auto& p2 = cone.front();
        const float distance = sqrt(pow(std::get<0>(p1) - std::get<0>(p2), 2) +
                                    pow(std::get<1>(p1) - std::get<1>(p2), 2));
        if (distance <= threshold) {
          cone.push_back(p1);
          found_cone = true;
          break;
        }
      }
      if (!found_cone) {
        cones.emplace_back(std::vector<std::tuple<float, float>>{p1});
      }
    }

    // Create a pose array message to store all the cones' locations.
    geometry_msgs::msg::PoseArray cones_locations;
    cones_locations.header.stamp = this->now();
    cones_locations.header.frame_id = "map";

    // find the average location of each cone.
    for (const auto& cone : cones) {
        float avg_x = 0.0, avg_y = 0.0;
      for (const auto& p : cone) {
        avg_x += std::get<0>(p);
        avg_y += std::get<1>(p);
      }
      avg_x /= cone.size();
      avg_y /= cone.size();

      //Create a pose message for the cone location.
      geometry_msgs::msg::Pose pose;
      pose.position.x = avg_x;
      pose.position.y = avg_y;
      pose.orientation.w = 1.0;

      // Add the cone location to the pose array message.
      cones_locations.poses.push_back(pose);
    }

    // Publish cone locations
    if (!cones_locations.poses.empty()) {
      cone_location_pub_->publish(cones_locations);
    }

  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr cone_location_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}