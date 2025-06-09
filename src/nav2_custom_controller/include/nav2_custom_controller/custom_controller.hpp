/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Template Author: John Cahill
 *  Student Version: Template Controller Header
 */

#ifndef NAV2_CUSTOM_CONTROLLER__CUSTOM_CONTROLLER_HPP_
#define NAV2_CUSTOM_CONTROLLER__CUSTOM_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>

// Core Nav2 and ROS2 includes
#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_custom_controller
{

/**
 * @brief Template CustomController for student implementation.
 *        Inherit from nav2_core::Controller and implement computeVelocityCommands.
 */
class CustomController : public nav2_core::Controller
{
public:
  CustomController() = default;
  ~CustomController() override = default;

  /// Plugin configuration, called on node bring-up
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  /**
   * @brief Core function students must implement.
   *        Returns the velocity command based on current pose and velocity.
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /// Receive the global plan from Nav2 planner
  void setPlan(const nav_msgs::msg::Path & path) override;

  /// Optional: Enforce a speed limit on the controller
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief Helper function to transform pose between frames
   */
  bool transformPose(
    const std::string & target_frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose,
    const rclcpp::Duration & transform_tolerance);

  /**
   * @brief Helper: Normalize any angle to [-pi, pi]
   */
  double normalizeAngle(double angle);

  /**
   * @brief Helper: Euclidean distance between two poses
   */
  double euclideanDistance(
    const geometry_msgs::msg::Pose & a,
    const geometry_msgs::msg::Pose & b);

  // === Core ROS2 and Nav2 Interfaces ===
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("CustomController")};
  rclcpp::Clock::SharedPtr clock_;

  // === Planning Interfaces ===
  nav_msgs::msg::Path global_plan_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  bool reached_position_ = false;

  // === STUDENT SECTION: Add and tune control gains as needed ===
  // Example: Nonlinear control gains (can be removed/renamed)
  double kp_;    // Proportional gain for position error
  double kpo_;   // Orientation proportional gain
  double k1_;    // Gain 1 - interpretation up to student
  double k2_;    // Gain 2 - interpretation up to student
  double k3_;    // Gain 3 - interpretation up to student
  double kpof_;  // Orientation feedforward gain (optional)

  // === STUDENT SECTION: Velocity tuning ===
  double max_linear_vel_;    // Max linear velocity
  double max_angular_vel_;   // Max angular velocity

  // === STUDENT SECTION: Lookahead logic if applicable ===
  double lookahead_dist_;    // Lookahead distance for trajectory following
};

}  // namespace nav2_custom_controller

#endif  // NAV2_CUSTOM_CONTROLLER__CUSTOM_CONTROLLER_HPP_

