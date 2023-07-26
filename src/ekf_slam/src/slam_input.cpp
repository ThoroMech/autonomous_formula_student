#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <cmath>

using namespace std::chrono_literals;

class GssSubscriber : public rclcpp::Node {
public:
  GssSubscriber() : Node("slam_input") 
  {
    gps_subscription = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps", 10, std::bind(&GssSubscriber::gps_callback, this, std::placeholders::_1));
    imu_subscription = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10, std::bind(&GssSubscriber::imu_callback, this, std::placeholders::_1));
    cones_subscription = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/cone_location", 10, std::bind(&GssSubscriber::cones_callback, this, std::placeholders::_1));
    map_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("map", 10);

    RCLCPP_INFO(this->get_logger(), "Slam Input Algorithm Initialized");
  }

private:
  //Create a pose array to store the map and a pose for the vehicle
  geometry_msgs::msg::Pose vehicle_pose;
  geometry_msgs::msg::PoseArray map;

  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) 
  {
    std::pair<double, double> vehicle_coordinate = coordinate2cartesian(msg->longitude, msg->latitude);
    vehicle_pose.position.x = vehicle_coordinate.first;
    vehicle_pose.position.y = vehicle_coordinate.second;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) 
  {
    // Compute yaw (Z-axis rotation) in radians
    vehicle_pose.orientation.z = std::atan2(2.0 * (msg->orientation.z * msg->orientation.w),
                           1.0 - 2.0 * (msg->orientation.z * msg->orientation.z));
  }

  void cones_callback(const geometry_msgs::msg::PoseArray msg)
  {
    bool ismatch;
    geometry_msgs::msg::PoseArray global_cone_location;
    geometry_msgs::msg::Pose traffic_cone;
    map.poses[0] = vehicle_pose;

    //convert the perceived traffic cone to global coordinate. 
    for (const auto& cone : msg.poses) //for each of the percieved cones
    {
      traffic_cone = convert_to_global(cone.position.x, cone.position.y);
      global_cone_location.poses.push_back(traffic_cone);
    }

    // //If the map is empty add all percieved cones
    // if (map.poses.size()==0){
    //   for (const auto& cone : global_cone_location.poses)
    //   {
    //     map.poses.push_back(cone);
    //     RCLCPP_INFO(this->get_logger(), "%ld number of cones found", map.poses.size());
    //   }
    // }
    //Otherwise check if any of the cones are new
    //else {
      for (const auto& cone : global_cone_location.poses) //for each of the percieved cones
      {
        ismatch = false;
        for (const auto& stored_cones : map.poses) //Check all stored locations
        {
          if (std::abs(cone.position.x - stored_cones.position.x)<0.3 && 
          std::abs(cone.position.y - stored_cones.position.y)<0.3)
          {
            ismatch = true;      
          }
        }
        if (ismatch == false){
          // Add the cone location to the map.
          map.poses.push_back(cone);
          RCLCPP_INFO(this->get_logger(), "%ld number of cones found", map.poses.size());
        }        
      }
    //}
    map_publisher->publish(map);
   }

  geometry_msgs::msg::Pose convert_to_global(double cone_x, double cone_y)
  {
    geometry_msgs::msg::Pose global_cone;
    global_cone.position.x = vehicle_pose.position.x + cone_x*cos(vehicle_pose.orientation.z) + cone_y*sin(vehicle_pose.orientation.z);
    global_cone.position.y = vehicle_pose.position.y + cone_x*sin(vehicle_pose.orientation.z) + cone_y*cos(vehicle_pose.orientation.z);
    return global_cone;
  }

  std::pair<double,double> coordinate2cartesian(double longitude, double latitude)
  {
    // Constants
    const double R = 6366627.0; 
    const double start_longitude = -122.140165;
    const double end_latitude = 47.641468;

    longitude = longitude - start_longitude;
    latitude = latitude - end_latitude;

    // Convert latitude and longitude to radians
    double latitude_rad = deg2rad(latitude);
    double longitude_rad = deg2rad(longitude);

    // Perform the Mercator projection
    double x = R * longitude_rad;
    double y = R * std::log(std::tan(M_PI / 4 + latitude_rad / 2));

    return std::make_pair(x, y);
  }
 
  double deg2rad(double degrees) 
  {
    return degrees * M_PI / 180.0;
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cones_subscription;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr map_publisher;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GssSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
