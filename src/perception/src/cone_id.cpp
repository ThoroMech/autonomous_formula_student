#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "shape_msgs/msg/plane.hpp"

#include "math.h"
#include <random>

/* This node is set up to subscribe to the Lidar2 topic.
I have adjusted the settings of Lidar2 to match the valodyne puck vlp-16
as this is what is available at USQ.
Initial observations suggest setting vlp-16 at 600RPM = 10Hz giving a resolution of 0.2 = 900 points per channel with 180 FOV.
the VLP-16 has 16 channels with 2 degree vertical resolution.
Lidar2 settings need to be changed in the settings.json file in "Formula-Student-Driverless-Simulator" folder:

"Lidar2": {
        "SensorType": 6,
        "Enabled": true,
        "X": 1.20, "Y": 0, "Z": 0.2,
        "Roll": 0, "Pitch": 0, "Yaw" : 0,
        "NumberOfLasers": 16,
        "PointsPerScan": 14400,
        "VerticalFOVUpper": 15,
        "VerticalFOVLower": -15,
        "HorizontalFOVStart": -90,
        "HorizontalFOVEnd": 90,
        "RotationsPerSecond": 10,
        "DrawDebugPoints": false
*/

class LidarSubscriber : public rclcpp::Node {
public:
  LidarSubscriber() : Node("lidar_subscriber"), 
                      lower_limit_(0.02),
                      upper_limit_(1.0),
                      cluster_threshold_(0.2), //20cm was found to be best to avoid cone duplication
                      cluster_proximity_ (0.5),
                      cluster_distance_ (0.5)
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar/Lidar2", 10, std::bind(&LidarSubscriber::topic_callback, this, std::placeholders::_1));
    cone_location_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("cone_location", 10);

    RCLCPP_INFO(this->get_logger(), "Perception Software - Cone Identification Initialised");
  }

private:
  float lower_limit_, upper_limit_, cluster_threshold_, cluster_proximity_, cluster_distance_;

  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Filter (x, y, z) coordinates from PointCloud2 message.
    std::vector<std::tuple<float, float, float>> points = filter_points(msg);
    
    // Calculate a ground plane
    shape_msgs::msg::Plane ground_plane = plane_calc(points);
    
    // Filter lidar points on ground plane or too high above ground plane
    std::vector<std::tuple<float, float, float>> candidate_points = filter_ground_air_points(points, ground_plane);

    //Cluster candidate points based on a threshold
    std::vector<std::vector<std::tuple<float, float, float>>> clusters = cluster_points(candidate_points);
    
    //Filter clusters if larger than traffic cone or too close to another cluster (walls etc)
    std::vector<std::vector<std::tuple<float, float, float>>> cones = filter_clusters(clusters);

    //average the points in clusters to determine cone location and write as PoseArray for publishing
    geometry_msgs::msg::PoseArray cones_locations = average_cone_location(cones);

    // Publish cone locations
    if (!cones_locations.poses.empty()) {
      cone_location_pub_->publish(cones_locations);
    }
    
  }
  
  std::vector<std::vector<std::tuple<float, float, float>>> filter_clusters(
        const std::vector<std::vector<std::tuple<float, float, float>>>& clusters) 
  {
    std::vector<std::vector<std::tuple<float, float, float>>> cones;

    // Iterate over all clusters and check if they satisfy the distance and proximity constraints
    for (const auto& cluster : clusters) {
        bool should_add_cluster = true;

        // Check if the cluster is too close to another cluster
        for (const auto& cone : cones) {
            if (std::hypot(
                std::get<0>(cone[0]) - std::get<0>(cluster[0]),
                std::get<1>(cone[0]) - std::get<1>(cluster[0]))
                    < this->cluster_proximity_
            ) {
                should_add_cluster = false;
                break;
            }
        }
        if (!should_add_cluster) {
            continue;
        }

        // Check if the distance between the furthest points in the cluster is too great
        float max_distance = 0;
        for (const auto& point1 : cluster) {
            for (const auto& point2 : cluster) {
                float distance = std::hypot(
                    std::get<0>(point1) - std::get<0>(point2),
                    std::get<1>(point1) - std::get<1>(point2)
                );
                if (distance > max_distance) {
                    max_distance = distance;
                }
            }
        }
        if (max_distance > this->cluster_distance_) {
            should_add_cluster = false;
        }

        // Add the cluster to the cones vector if it satisfies the constraints
        if (should_add_cluster) {
            cones.push_back(cluster);
        }
    }

    return cones;
  }

  geometry_msgs::msg::PoseArray average_cone_location(const std::vector<std::vector<std::tuple<float, float, float>>>& cones)
  {
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
    return cones_locations;
  }

  std::vector<std::vector<std::tuple<float, float, float>>> cluster_points(const std::vector<std::tuple<float, float, float>>& candidate_points)
  {
    std::vector<std::vector<std::tuple<float, float, float>>> clusters;
    for (const auto& p1 : candidate_points) {
      bool found_cluster = false;
      for (auto& cluster : clusters) {
        const auto& p2 = cluster.front();
        const float distance = sqrt(pow(std::get<0>(p1) - std::get<0>(p2), 2) +
                                    pow(std::get<1>(p1) - std::get<1>(p2), 2));
        if (distance <= this->cluster_threshold_) {
          cluster.push_back(p1);
          found_cluster = true;
          break;
        }
      }
      if (!found_cluster) {
        clusters.emplace_back(std::vector<std::tuple<float, float, float>>{p1});
      }
    }
    return clusters;
  }

  std::vector<std::tuple<float, float, float>> filter_ground_air_points(const std::vector<std::tuple<float, float, float>>& points,
                                                                        shape_msgs::msg::Plane ground_plane)
  {
    std::vector<std::tuple<float, float, float>> candidate_points;
    for (auto& point : points) {
      float distance = distance_to_plane(point, ground_plane);
      if (distance > this->lower_limit_ && distance < this->upper_limit_){
        candidate_points.emplace_back(point);
      }
    }
    return candidate_points;
  }

  shape_msgs::msg::Plane plane_calc(const std::vector<std::tuple<float, float, float>>& points)
  {
    int iterations = 10;
    double best_error = 0;
    shape_msgs::msg::Plane best_ground_plane;

    for (int i = 0; i < iterations; i++){
      //sample 500 points
      std::vector<std::tuple<float, float, float>> sample_points = sampler(500, points);

      //Sample 3 points
      std::vector<std::tuple<float, float, float>> plane_sample = sampler(3, sample_points);

      //Fit plane to 3 points
      shape_msgs::msg::Plane ground_plane = fit_plane(plane_sample);

      //Calculate error of sample points to ground plane
      double error = error_calc(sample_points, ground_plane);
      
      //If error is smallest for iterations save error and ground plane
      if (best_error == 0 || error < best_error){
        best_error = error;
        best_ground_plane = ground_plane;
      } 
    }
    return best_ground_plane;
  }

  double distance_to_plane(std::tuple<float, float, float> point, shape_msgs::msg::Plane plane)
  {
    double a = plane.coef[0];
    double b = plane.coef[1];
    double c = plane.coef[2];
    double d = plane.coef[3];
    float x = std::get<0>(point);
    float y = std::get<1>(point);
    float z = std::get<2>(point);
    double distance = std::abs(a * x + b * y + c * z + d) / std::sqrt(a * a + b * b + c * c);
    return distance;
  }

  double error_calc(std::vector<std::tuple<float, float, float>> plane_sample, shape_msgs::msg::Plane ground_plane) 
  {
      double total_distance = 0.0;
      for (auto& point : plane_sample) {
          double distance = distance_to_plane(point, ground_plane);
          total_distance += distance;
      }
      return total_distance;
  }

  shape_msgs::msg::Plane fit_plane(const std::vector<std::tuple<float, float, float>>& points) 
  {
      // Extract the coordinates of the three points.
      const auto& [x1, y1, z1] = points[0];
      const auto& [x2, y2, z2] = points[1];
      const auto& [x3, y3, z3] = points[2];

      // Compute two vectors that lie on the plane.
      geometry_msgs::msg::Vector3 u, v, n;
      //Compute vector u
      u.x = x2 - x1;
      u.y = y2 - y1;
      u.z = z2 - z1;
      //Compute vector v
      v.x = x3 - x1;
      v.y = y3 - y1;
      v.z = z3 - z1;
      // Compute the normal vector to the plane by taking the cross product of u and v.
      n.x = u.y * v.z - u.z * v.y;
      n.y = u.z * v.x - u.x * v.z;
      n.z = u.x * v.y - u.y * v.x;

      // Normalize the normal vector.
      const auto norm = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
      const auto nx = n.x / norm, ny = n.y / norm, nz = n.z / norm;

      // Compute the distance from the plane to the origin.
      const auto d = -(nx * x1 + ny * y1 + nz * z1);

      // Create the plane message.
      shape_msgs::msg::Plane plane;
      plane.coef[0] = nx;
      plane.coef[1] = ny;
      plane.coef[2] = nz;
      plane.coef[3] = d;

      return plane;
  }

  std::vector<std::tuple<float, float, float>> sampler(const int sample_size, const std::vector<std::tuple<float, float, float>> points)
  //Function to take a random sample of sample_size
  {
    std::vector<std::tuple<float, float, float>> sample_points;
    if (points.size() > 0) {
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_int_distribution<> distrib(0, points.size() - 1);
      for (int i = 0; i < sample_size; i++) {
        sample_points.push_back(points[distrib(gen)]);
      }
    }
    return sample_points;
  }
  
  std::vector<std::tuple<float, float, float>> filter_points(const sensor_msgs::msg::PointCloud2::SharedPtr msg) 
  {
    // Convert PointCloud2 to xyz coordinates.
    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

    // Create a vector of tuples to store (x, y, z) coordinates and distance.
    std::vector<std::tuple<float, float, float>> points;

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;
      if (x != 0 && y != 0 && z != 0){
        // Add the (x, y, z) coordinates to the vector.
        points.emplace_back(x, y, z);
      }
    }
    return points;
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