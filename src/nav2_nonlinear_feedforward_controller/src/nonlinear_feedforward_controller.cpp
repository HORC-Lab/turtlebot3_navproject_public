#include "nav2_nonlinear_feedforward_controller/nonlinear_feedforward_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_core/exceptions.hpp"
#include <cmath>
#include <algorithm>

namespace nav2_nonlinear_feedforward_controller
{

using std::min;
using std::max;

// Configure method to initialize controller parameters from ROS2 parameter server
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

  // Declare and retrieve control parameters
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

// Lifecycle hooks (no operations in this example)
void NonlinearFeedforwardController::cleanup() {}
void NonlinearFeedforwardController::activate() {}
void NonlinearFeedforwardController::deactivate() {}

// Set the global plan for the controller to follow
void NonlinearFeedforwardController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  goal_pose_ = path.poses.back(); // Save final goal pose
  reached_position_ = false; // Reset state
}

// Normalize angle to range [-pi, pi]
double NonlinearFeedforwardController::normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// Calculate Euclidean distance between two poses
double NonlinearFeedforwardController::euclideanDistance(
  const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
{
  return std::hypot(a.position.x - b.position.x, a.position.y - b.position.y);
}

// Transform a pose to the specified target frame
bool NonlinearFeedforwardController::transformPose(
  const std::string & target_frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  const rclcpp::Duration & /*transform_tolerance*/)
{
  if (in_pose.header.frame_id == target_frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, target_frame);
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Transform error: %s", ex.what());
    return false;
  }
}

// Main control method to compute velocity commands based on current pose
geometry_msgs::msg::TwistStamped NonlinearFeedforwardController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  const auto base_frame = costmap_ros_->getBaseFrameID();
  const auto global_frame = global_plan_.header.frame_id;

  // Transform robot pose into global frame
  geometry_msgs::msg::PoseStamped robot_pose_in_global;
  if (!transformPose(global_frame, pose, robot_pose_in_global, rclcpp::Duration::from_seconds(0.1))) {
    throw nav2_core::PlannerException("Failed to transform robot pose to global plan frame");
  }

  // Check if the robot has reached the goal position
  if (!reached_position_ &&
      euclideanDistance(robot_pose_in_global.pose, goal_pose_.pose) < 0.10) {
    reached_position_ = true;
  }

  // If goal position is reached, align robot orientation
  if (reached_position_) {
    double goal_yaw = tf2::getYaw(goal_pose_.pose.orientation);
    double current_yaw = tf2::getYaw(robot_pose_in_global.pose.orientation);
    double phi_error = normalizeAngle(goal_yaw - current_yaw);
    double w = kpof_ * phi_error * std::exp(-std::abs(phi_error)); // Angular velocity command
    w = max(min(w, max_angular_vel_), -max_angular_vel_);

    if (std::abs(phi_error) < 0.05) {
      w = 0.0; // Stop rotating if close to desired orientation
      RCLCPP_INFO(logger_, "Final orientation reached. Stopping rotation.");
    }

    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.header.frame_id = base_frame;
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = w;
    return cmd_vel;
  }

  // Determine lookahead pose from global plan
  geometry_msgs::msg::PoseStamped lookahead_pose_in_global;
  lookahead_pose_in_global.pose = goal_pose_.pose;
  lookahead_pose_in_global.header.frame_id = global_frame;
  lookahead_pose_in_global.header.stamp = pose.header.stamp;

  for (const auto & ps : global_plan_.poses) {
    if (euclideanDistance(ps.pose, robot_pose_in_global.pose) > lookahead_dist_) {
      lookahead_pose_in_global.pose = ps.pose;
      break;
    }
  }

  // Transform lookahead pose to robot base frame
  geometry_msgs::msg::PoseStamped lookahead_pose;
  if (!transformPose(base_frame, lookahead_pose_in_global, lookahead_pose, rclcpp::Duration::from_seconds(0.1))) {
    throw nav2_core::PlannerException("Failed to transform lookahead pose to base frame");
  }

  // Extract relative pose components in base frame
  const double xd = lookahead_pose.pose.position.x;
  const double yd = lookahead_pose.pose.position.y;
  const double phid = std::atan2(yd, xd); // Desired heading
  const double phi = 0.0; // Robot heading assumed to be zero in base frame

  // Calculate translational velocity component
  const double dist = std::hypot(xd, yd);
  const double vd = std::min(kp_ * dist, max_linear_vel_);

  // Compute errors and feedback terms
  double phi_e = normalizeAngle(phid - phi);
  double x_e = std::cos(phid) * (-xd) + std::sin(phid) * (-yd);
  double y_e = -std::sin(phid) * (-xd) + std::cos(phid) * (-yd);

  double tan_phi_e = std::tan(phi_e);
  if (std::abs(tan_phi_e) > 3.0) {
    tan_phi_e = 3.0 * ((tan_phi_e > 0) ? 1 : -1); // Clamp tangent
  }

  double cos_phi_e = std::max(std::cos(phi_e), 0.1); // Avoid divide by zero
  double v = (vd - k1_ * std::abs(vd) * (x_e + y_e * tan_phi_e)) / cos_phi_e; // Linear velocity
  double wd = kpo_ * phi_e; // Orientation feedback term
  double w = wd - (k2_ * vd * y_e + k3_ * std::abs(vd) * tan_phi_e) * cos_phi_e * cos_phi_e; // Angular velocity

  // Clamp velocities to max limits
  v = max(min(v, max_linear_vel_), 0.0);
  w = max(min(w, max_angular_vel_), -max_angular_vel_);

  // Return computed velocity command
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.header.frame_id = base_frame;
  cmd_vel.twist.linear.x = v;
  cmd_vel.twist.angular.z = w;
  return cmd_vel;
}

// Method to set speed limit (not used in this implementation)
void NonlinearFeedforwardController::setSpeedLimit(
  const double & /*speed_limit*/, const bool & /*percentage*/)
{
  // No-op
}

}  // namespace nav2_nonlinear_feedforward_controller

// Export controller plugin
PLUGINLIB_EXPORT_CLASS(nav2_nonlinear_feedforward_controller::NonlinearFeedforwardController, nav2_core::Controller)

