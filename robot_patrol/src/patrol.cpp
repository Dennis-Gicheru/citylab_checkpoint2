#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>
#include <cmath>

class Patrol : public rclcpp::Node
{
public:
  Patrol() : Node("patrol_node"), direction_(0.0), obstacle_detected_(false)
  {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&Patrol::scan_callback, this, std::placeholders::_1));

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&Patrol::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Patrol node has started.");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const int total_rays  = static_cast<int>(msg->ranges.size());
    const int start_idx   = total_rays / 4;       // -π/2
    const int end_idx     = 3 * total_rays / 4;   // +π/2
    const int center_idx  = total_rays / 2;        // 0 rad (forward)

    obstacle_detected_ = false;
    float max_dist = 0.0f;

    for (int i = start_idx; i < end_idx; ++i) {
      const float range = msg->ranges[i];

      if (!std::isfinite(range)) continue;

      // Obstacle check — narrow front cone (±10 rays around center)
      if (std::abs(i - center_idx) <= 10 && range < 0.35f) {
        obstacle_detected_ = true;
      }

      // Track the ray with the greatest valid distance
      if (range > max_dist) {
        max_dist    = range;
        direction_  = msg->angle_min + (i * msg->angle_increment);
      }
    }

    // No obstacle — reset steering to drive straight
    if (!obstacle_detected_) {
      direction_ = 0.0f;
    }
  }

  void control_loop()
  {
    auto msg          = geometry_msgs::msg::Twist();
    msg.linear.x      = 0.1f;
    msg.angular.z     = obstacle_detected_ ? direction_ / 2.0f : 0.0f;
    vel_pub_->publish(msg);
  }

  float  direction_;
  bool   obstacle_detected_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      vel_pub_;
  rclcpp::TimerBase::SharedPtr                                  timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}