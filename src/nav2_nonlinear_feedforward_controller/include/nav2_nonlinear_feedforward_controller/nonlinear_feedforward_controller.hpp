/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Author(s): John Cahill <johncahill4493@gmail.com>
 */

#ifndef NAV2_NONLINEAR_FEEDFORWARD_CONTROLLER__NONLINEAR_FEEDFORWARD_CONTROLLER_HPP_
#define NAV2_NONLINEAR_FEEDFORWARD_CONTROLLER__NONLINEAR_FEEDFORWARD_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_nonlinear_feedforward_controller
{

// Controller class implementing nonlinear feedforward logic for robot motion
class NonlinearFeedforwardController : public nav2_core::Controller
{
public:
  NonlinearFeedforwardController() = default;
  ~NonlinearFeedforwardController() override = default;

  // Configuration lifecycle method to initialize controller settings
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // Lifecycle hooks (called during navigation lifecycle transitions)
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  // Main method to compute velocity commands for robot motion
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  // Method to provide the controller with the global plan
  void setPlan(const nav_msgs::msg::Path & path) override;

  // Optional speed limit setter (not implemented)
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  // Utility method to transform pose between coordinate frames
  bool transformPose(
    const std::string & target_frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose,
    const rclcpp::Duration & transform_tolerance);

protected:
  // Normalize angle to range [-pi, pi]
  double normalizeAngle(double angle);

  // Compute Euclidean distance between two poses
  double euclideanDistance(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b);

  // ROS2 node, transform buffer, and costmap handle
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  // Name of the plugin instance
  std::string plugin_name_;

  // Logger and clock utilities
  rclcpp::Logger logger_{rclcpp::get_logger("NonlinearFeedforwardController")};
  rclcpp::Clock::SharedPtr clock_;

  // Current global plan and final goal pose
  nav_msgs::msg::Path global_plan_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  bool reached_position_ = false;

  // Control gain parameters
  double kp_;
  double kpo_;
  double k1_;
  double k2_;
  double k3_;
  double kpof_;

  // Velocity limits
  double max_linear_vel_;
  double max_angular_vel_;
  
  // Lookahead distance used for local goal selection
  double lookahead_dist_;
};

}  // namespace nav2_nonlinear_feedforward_controller

#endif  // NAV2_NONLINEAR_FEEDFORWARD_CONTROLLER__NONLINEAR_FEEDFORWARD_CONTROLLER_HPP_

