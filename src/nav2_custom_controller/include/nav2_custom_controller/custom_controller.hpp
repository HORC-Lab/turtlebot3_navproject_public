/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
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

class CustomController : public nav2_core::Controller
{
public:
  CustomController() = default;
  ~CustomController() override = default;

  // === [BLACK BOX] Plugin configuration, called during lifecycle setup
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // === [BLACK BOX] Lifecycle hooks (not used in this template)
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  // === [STUDENT SECTION] Main control logic: called each cycle to compute velocity
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  // === [BLACK BOX] Called when a new global plan is received
  void setPlan(const nav_msgs::msg::Path & path) override;

  // === [BLACK BOX] Optional: Enforce speed limits (not used here)
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  // === [BLACK BOX] Helper: Transform pose between coordinate frames
  bool transformPose(
    const std::string & target_frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose,
    const rclcpp::Duration & transform_tolerance);

  // === [BLACK BOX] ROS2 and Nav2 interfaces
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("CustomController")};
  rclcpp::Clock::SharedPtr clock_;

  // === [BLACK BOX] Planning data from Nav2
  nav_msgs::msg::Path global_plan_;
  geometry_msgs::msg::PoseStamped goal_pose_;

  // === [STUDENT SECTION] Example parameter (students may add more)
  double kp_;  // Proportional gain for goal distance

  // === [STUDENT SECTION] Add any custom helper functions below (declarations only)
  // Example:
  // double normalizeAngle(double angle);
  // double euclideanDistance(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b);
};

}  // namespace nav2_custom_controller

#endif  // NAV2_CUSTOM_CONTROLLER__CUSTOM_CONTROLLER_HPP_

