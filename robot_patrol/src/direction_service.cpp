#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

using GetDirection = robot_patrol::srv::GetDirection;
using std::placeholders::_1;
using std::placeholders::_2;

class DirectionService : public rclcpp::Node
{
public:
  DirectionService() : Node("direction_service_node")
  {
    service_ = this->create_service<GetDirection>(
      "/direction_service",
      std::bind(&DirectionService::direction_callback, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "Service Server Ready");
  }

private:
  // Derive a scan array index from a real-world angle (hardware-agnostic).
  int angle_to_index(const sensor_msgs::msg::LaserScan & scan, float angle) const
  {
    const int total = static_cast<int>(scan.ranges.size());
    int idx = static_cast<int>(std::round((angle - scan.angle_min) / scan.angle_increment));
    return std::max(0, std::min(idx, total - 1));
  }

  void direction_callback(
    const std::shared_ptr<GetDirection::Request> request,
    std::shared_ptr<GetDirection::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Request Received");

    const auto & scan = request->laser_data;

    // Three 60° sectors across the front 180°
    const int front_end  = angle_to_index(scan, M_PI / 6.0f);    // 30°
    const int left_end   = angle_to_index(scan, M_PI);           // 180°
    const int right_start = left_end;
    const int right_end  = scan.ranges.size();
 
    float total_right = 0.0f;
    float total_front = 0.0f;
    float total_left  = 0.0f;
    float min_front   = std::numeric_limits<float>::infinity();

     // Front sector: [0°, 30°)
    for (int i = 0; i < front_end; ++i) {
      const float r = scan.ranges[i];
      if (std::isfinite(r)) {
        total_front += r;
        min_front = std::min(min_front, r);
      }
    }

    // Left sector: [30°, 180°)
    for (int i = front_end; i < left_end; ++i) {
      const float r = scan.ranges[i];
      if (std::isfinite(r)) total_left += r;
    }

    // Right sector: [180°, 360°)
    for (int i = right_start; i < right_end; ++i) {
      const float r = scan.ranges[i];
      if (std::isfinite(r)) total_right += r;
    }

    // Decision logic (per spec)
    std::string direction;
    if (min_front > 0.50f) {
      direction = "forward";
    } else if (total_left > total_right) {
      direction = "left";
    } else {
      direction = "right";
    }

    response->direction = direction;

    RCLCPP_INFO(this->get_logger(),
      "Request Completed | front_min=%.2f R=%.1f F=%.1f L=%.1f -> %s",
      min_front, total_right, total_front, total_left, direction.c_str());
  }

  rclcpp::Service<GetDirection>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}