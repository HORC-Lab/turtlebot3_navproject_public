#include "nav2_nonlinear_feedforward_controller/nonlinear_feedforward_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>
#include <algorithm>

namespace nav2_nonlinear_feedforward_controller
{

using std::min;
using std::max;

void NonlinearFeedforwardController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  clock_ = node_->get_clock();
  logger_ = node_->get_logger();

  // Declare and get parameters
  node_->declare_parameter(name + ".kp", 0.4);
  node_->declare_parameter(name + ".kpo", 1.8);
  node_->declare_parameter(name + ".k1", 0.2);
  node_->declare_parameter(name + ".k2", 3.0);
  node_->declare_parameter(name + ".k3", 5.0);
  node_->declare_parameter(name + ".kpof", 2.8);
  node_->declare_parameter(name + ".max_linear_vel", 0.22);
  node_->declare_parameter(name + ".max_angular_vel", 2.84);
  node_->declare_parameter(name + ".lookahead_dist", 0.3);

  node_->get_parameter(name + ".kp", kp_);
  node_->get_parameter(name + ".kpo", kpo_);
  node_->get_parameter(name + ".k1", k1_);
  node_->get_parameter(name + ".k2", k2_);
  node_->get_parameter(name + ".k3", k3_);
  node_->get_parameter(name + ".kpof", kpof_);
  node_->get_parameter(name + ".max_linear_vel", max_linear_vel_);
  node_->get_parameter(name + ".max_angular_vel", max_angular_vel_);
  node_->get_parameter(name + ".lookahead_dist", lookahead_dist_);
}

void NonlinearFeedforwardController::cleanup() {}
void NonlinearFeedforwardController::activate() {}
void NonlinearFeedforwardController::deactivate() {}

void NonlinearFeedforwardController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  goal_pose_ = path.poses.back();
  reached_position_ = false;
}

double NonlinearFeedforwardController::normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double NonlinearFeedforwardController::euclideanDistance(
  const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
{
  return std::hypot(a.position.x - b.position.x, a.position.y - b.position.y);
}

geometry_msgs::msg::TwistStamped NonlinearFeedforwardController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  const auto & x = pose.pose.position.x;
  const auto & y = pose.pose.position.y;
  const auto & phi = tf2::getYaw(pose.pose.orientation);

  // Select lookahead target
  geometry_msgs::msg::Pose tracking_pose = goal_pose_.pose;
  for (const auto & ps : global_plan_.poses) {
    if (euclideanDistance(ps.pose, pose.pose) > lookahead_dist_) {
      tracking_pose = ps.pose;
      break;
    }
  }

  const auto & xd = tracking_pose.position.x;
  const auto & yd = tracking_pose.position.y;
  const double phid = std::atan2(yd - y, xd - x);  // Desired heading

  // Feedforward components
  const double dx = xd - x;
  const double dy = yd - y;
  const double dist = std::hypot(dx, dy);
  const double vd = std::min(kp_ * dist, max_linear_vel_);

  // FIX 1: Correct heading error direction
  double phi_e = normalizeAngle(phid - phi);

  // Transform error into tracking frame
  double x_e = std::cos(phid) * (x - xd) + std::sin(phid) * (y - yd);
  double y_e = -std::sin(phid) * (x - xd) + std::cos(phid) * (y - yd);

  // FIX 2: Clamp tan(phi_e) to avoid spikes
  double tan_phi_e = std::tan(phi_e);
  if (std::abs(tan_phi_e) > 3.0) {
    tan_phi_e = 3.0 * ((tan_phi_e > 0) ? 1 : -1);
  }

  // FIX 3: Add floor to cos(phi_e)
  double cos_phi_e = std::cos(phi_e);
  cos_phi_e = std::max(cos_phi_e, 0.1);

  // FIX 4: Smooth velocity reduction using heading error
  double v = (vd - k1_ * std::abs(vd) * (x_e + y_e * tan_phi_e)) / cos_phi_e;

  // Nonlinear angular correction
  double wd = kpo_ * phi_e;
  double w = wd - (k2_ * vd * y_e + k3_ * std::abs(vd) * tan_phi_e) * cos_phi_e * cos_phi_e;

  // Clamp
  v = max(min(v, max_linear_vel_), 0.0);
  w = max(min(w, max_angular_vel_), -max_angular_vel_);

  // Goal behavior
  if (euclideanDistance(pose.pose, goal_pose_.pose) < 0.05) {
    v = 0.0;
    reached_position_ = true;
  }

  if (reached_position_) {
    double phi_error = normalizeAngle(tf2::getYaw(goal_pose_.pose.orientation) - phi);

    // FIX 5: Smooth final orientation
    w = kpof_ * phi_error * std::exp(-std::abs(phi_error));
    w = max(min(w, max_angular_vel_), -max_angular_vel_);

    if (std::abs(phi_error) < 0.001) {
      w = 0.0;
      RCLCPP_INFO(logger_, "Final orientation reached. Stopping rotation.");
    }
  }

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.twist.linear.x = v;
  cmd_vel.twist.angular.z = w;
  return cmd_vel;
}

void NonlinearFeedforwardController::setSpeedLimit(
  const double & /*speed_limit*/, const bool & /*percentage*/)
{
  // No-op
}

}  // namespace nav2_nonlinear_feedforward_controller

PLUGINLIB_EXPORT_CLASS(nav2_nonlinear_feedforward_controller::NonlinearFeedforwardController, nav2_core::Controller)

