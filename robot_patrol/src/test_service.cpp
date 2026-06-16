#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <chrono>
#include <memory>

using GetDirection = robot_patrol::srv::GetDirection;
using namespace std::chrono_literals;
using std::placeholders::_1;

class TestService : public rclcpp::Node
{
public:
  TestService() : Node("test_service_node")
  {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&TestService::scan_callback, this, _1));

    client_ = this->create_client<GetDirection>("/direction_service");

    timer_ = this->create_wall_timer(
      1s, std::bind(&TestService::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Client Ready");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    last_scan_ = msg;
  }

  void timer_callback()
  {
    if (!last_scan_) {
      RCLCPP_WARN(this->get_logger(), "Waiting for laser data...");
      return;
    }

    if (!client_->wait_for_service(1s)) {
      RCLCPP_ERROR(this->get_logger(), "/direction_service not available");
      return;
    }

    auto request = std::make_shared<GetDirection::Request>();
    request->laser_data = *last_scan_;

    RCLCPP_INFO(this->get_logger(), "Request Sent");
    client_->async_send_request(
      request,
      std::bind(&TestService::response_callback, this, _1));
  }

  void response_callback(rclcpp::Client<GetDirection>::SharedFuture future)
  {
    RCLCPP_INFO(this->get_logger(), "Response Received -> %s",
                future.get()->direction.c_str());
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Client<GetDirection>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestService>());
  rclcpp::shutdown();
  return 0;
}