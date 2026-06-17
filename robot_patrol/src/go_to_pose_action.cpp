#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_patrol/action/go_to_pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>

using GoToPoseAction = robot_patrol::action::GoToPose;
using GoalHandle     = rclcpp_action::ServerGoalHandle<GoToPoseAction>;
using std::placeholders::_1;
using std::placeholders::_2;

class GoToPose : public rclcpp::Node
{
public:
  GoToPose() : Node("go_to_pose_action_node")
  {
    // Odometry subscriber — keeps current_pos_ updated
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&GoToPose::odom_callback, this, _1));

    // Velocity publisher
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Action server
    action_server_ = rclcpp_action::create_server<GoToPoseAction>(
      this, "/go_to_pose",
      std::bind(&GoToPose::handle_goal,     this, _1, _2),
      std::bind(&GoToPose::handle_cancel,   this, _1),
      std::bind(&GoToPose::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "Action Server Ready");
  }

private:
  // ── Odometry → current_pos_ (x, y, yaw in radians) ──────────────────────────
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_pos_.x     = msg->pose.pose.position.x;
    current_pos_.y     = msg->pose.pose.position.y;
    current_pos_.theta = yaw;  // radians
  }

  // ── Action plumbing ─────────────────────────────────────────────────────────
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GoToPoseAction::Goal> goal)
  {
    (void)uuid;
    desired_pos_ = goal->goal_pos;  // store target (theta in degrees)
    RCLCPP_INFO(this->get_logger(), "Goal Received -> x=%.2f y=%.2f theta=%.1f deg",
                desired_pos_.x, desired_pos_.y, desired_pos_.theta);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Goal cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    // Run control loop off the executor thread
    std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
  }

  // ── Control loop ────────────────────────────────────────────────────────────
  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    rclcpp::Rate loop_rate(20);  // 20 Hz
    auto feedback = std::make_shared<GoToPoseAction::Feedback>();
    auto result   = std::make_shared<GoToPoseAction::Result>();

    const double goal_theta_rad = desired_pos_.theta * M_PI / 180.0;
    int fb_counter = 0;

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        stop_robot();
        result->status = false;
        goal_handle->canceled(result);
        return;
      }

      // Snapshot the current pose
      geometry_msgs::msg::Pose2D cur;
      {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        cur = current_pos_;
      }

      const double dx   = desired_pos_.x - cur.x;
      const double dy   = desired_pos_.y - cur.y;
      const double dist = std::hypot(dx, dy);

      geometry_msgs::msg::Twist cmd;

      if (dist > POS_TOL) {
        // Phase 1 — drive to the target position
        const double angle_to_goal = std::atan2(dy, dx);
        const double yaw_err = normalize_angle(angle_to_goal - cur.theta);

        cmd.angular.z = saturate(ANG_GAIN * yaw_err, -MAX_ANG, MAX_ANG);
        cmd.linear.x  = (std::fabs(yaw_err) < HEADING_TOL) ? FWD_SPEED : 0.0;
      } else {
        // Phase 2 — rotate to the final orientation
        const double theta_err = normalize_angle(goal_theta_rad - cur.theta);

        if (std::fabs(theta_err) > ORIENT_TOL) {
          cmd.angular.z = saturate(ANG_GAIN * theta_err, -MAX_ANG, MAX_ANG);
          cmd.linear.x  = 0.0;
        } else {
          break;  // both position and orientation satisfied
        }
      }

      vel_pub_->publish(cmd);

      // Feedback at ~1 Hz
      if (++fb_counter >= 20) {
        fb_counter = 0;
        feedback->current_pos.x     = cur.x;
        feedback->current_pos.y     = cur.y;
        feedback->current_pos.theta = cur.theta * 180.0 / M_PI;  // degrees
        goal_handle->publish_feedback(feedback);
      }

      loop_rate.sleep();
    }

    stop_robot();
    result->status = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal Completed");
  }

  void stop_robot()
  {
    geometry_msgs::msg::Twist stop;
    vel_pub_->publish(stop);  // all zeros
  }

  static double normalize_angle(double a)
  {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  static double saturate(double v, double lo, double hi)
  {
    return std::max(lo, std::min(v, hi));
  }

  // ── Tuning constants ────────────────────────────────────────────────────────
  static constexpr double POS_TOL     = 0.075;                  // m
  static constexpr double ORIENT_TOL  = 10.0 * M_PI / 180.0;    // rad (10°)
  static constexpr double HEADING_TOL = 0.15;                   // rad — forward-motion gate
  static constexpr double FWD_SPEED   = 0.2;                    // m/s
  static constexpr double ANG_GAIN    = 1.0;
  static constexpr double MAX_ANG     = 1.0;                    // rad/s

  // ── State ───────────────────────────────────────────────────────────────────
  geometry_msgs::msg::Pose2D desired_pos_;  // theta in degrees
  geometry_msgs::msg::Pose2D current_pos_;  // theta in radians
  std::mutex pose_mutex_;

  // ── ROS 2 handles ─────────────────────────────────────────────────────────────
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  vel_pub_;
  rclcpp_action::Server<GoToPoseAction>::SharedPtr         action_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoToPose>());
  rclcpp::shutdown();
  return 0;
}