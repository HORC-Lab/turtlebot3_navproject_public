/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Author(s): John Cahill <johncahill4493@gmail.com>
 */

#ifndef NAV2_CUSTOM_CONTROLLER__CUSTOM_CONTROLLER_HPP_
#define NAV2_CUSTOM_CONTROLLER__CUSTOM_CONTROLLER_HPP_

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

namespace nav2_custom_controller
{

class CustomController : public nav2_core::Controller
{
public:
  CustomController() = default;
  ~CustomController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  bool transformPose(
  	const std::string & target_frame,
  	const geometry_msgs::msg::PoseStamped & in_pose,
  	geometry_msgs::msg::PoseStamped & out_pose,
  	const rclcpp::Duration & transform_tolerance);

protected:
  double normalizeAngle(double angle);
  double euclideanDistance(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("CustomController")};
  rclcpp::Clock::SharedPtr clock_;

  nav_msgs::msg::Path global_plan_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  bool reached_position_ = false;

  // Control gains
  double kp_;
  double kpo_;
  double k1_;
  double k2_;
  double k3_;
  double kpof_;

  // Velocity limits
  double max_linear_vel_;
  double max_angular_vel_;
  
  // Lookahead distance
  double lookahead_dist_;
};

}  // namespace nav2_custom_controller

#endif  // NAV2_CUSTOM_CONTROLLER__CUSTOM_CONTROLLER_HPP_

