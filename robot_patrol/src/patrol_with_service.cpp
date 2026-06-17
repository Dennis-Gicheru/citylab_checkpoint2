#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <chrono>
#include <memory>
#include <string>

using GetDirection = robot_patrol::srv::GetDirection;
using namespace std::chrono_literals;
using std::placeholders::_1;

class Patrol : public rclcpp::Node
{
public:
  Patrol() : Node("patrol_with_service_node")
  {
    // ── Laser subscriber — stores the most recent scan ──────────────────────
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&Patrol::scan_callback, this, _1));

    // ── Velocity publisher ──────────────────────────────────────────────────
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // ── Service client for /direction_service ───────────────────────────────
    client_ = this->create_client<GetDirection>("/direction_service");

    // ── Control loop at 10 Hz ───────────────────────────────────────────────
    timer_ = this->create_wall_timer(
      100ms, std::bind(&Patrol::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Patrol (service mode) started.");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    last_scan_ = msg;
  }

  // ── Control loop ──────────────────────────────────────────────────────────
  // 1. Publish velocity based on the latest direction from the service.
  // 2. Fire a fresh async service request to refresh that direction.
  void control_loop()
  {
    auto cmd = geometry_msgs::msg::Twist();

    if (direction_ == "forward") {
      cmd.linear.x  = 0.1;
      cmd.angular.z = 0.0;
    } else if (direction_ == "left") {
      cmd.linear.x  = 0.1;
      cmd.angular.z = 0.5;
    } else if (direction_ == "right") {
      cmd.linear.x  = 0.1;
      cmd.angular.z = -0.5;
    } else {
      // No direction yet — stay still until the first response arrives
      cmd.linear.x  = 0.0;
      cmd.angular.z = 0.0;
    }

    vel_pub_->publish(cmd);

    // Refresh the direction asynchronously (no blocking, no deadlock)
    request_direction();
  }

  void request_direction()
  {
    if (!last_scan_ || service_in_flight_) {
      return;  // nothing to send yet, or a request is already pending
    }
    if (!client_->service_is_ready()) {
      return;  // server not up yet
    }

    auto request = std::make_shared<GetDirection::Request>();
    request->laser_data = *last_scan_;

    service_in_flight_ = true;
    client_->async_send_request(
      request,
      std::bind(&Patrol::direction_response, this, _1));
  }

  void direction_response(rclcpp::Client<GetDirection>::SharedFuture future)
  {
    direction_ = future.get()->direction;
    service_in_flight_ = false;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Direction: %s", direction_.c_str());
  }

  // ── State ─────────────────────────────────────────────────────────────────
  std::string direction_;            // latest decision from the service
  bool service_in_flight_ = false;   // guards against overlapping requests
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

  // ── ROS 2 handles ───────────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      vel_pub_;
  rclcpp::Client<GetDirection>::SharedPtr                      client_;
  rclcpp::TimerBase::SharedPtr                                 timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}