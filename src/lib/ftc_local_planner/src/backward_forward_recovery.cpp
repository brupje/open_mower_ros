#include <ftc_local_planner/backward_forward_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace ftc_local_planner
{

BackwardForwardRecovery::BackwardForwardRecovery()
  : initialized_(false),
    max_distance_(0.5),
    linear_vel_(0.3),
    check_frequency_(10.0),
    max_cost_threshold_(costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 10),
    timeout_(ros::Duration(3.0)),
    forward_check_distance_(0.6),
    has_xb_pose_(false) {}

void BackwardForwardRecovery::initialize(std::string name, tf2_ros::Buffer* tf,
                                         costmap_2d::Costmap2DROS* global_costmap,
                                         costmap_2d::Costmap2DROS* local_costmap)
{
  if (!initialized_)
  {
    name_ = name;
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    ros::NodeHandle private_nh("~/" + name_);
    // Publish on /nav_vel so twist_mux priority 10 is used (same as MBF nav_vel output)
    cmd_vel_pub_ = private_nh.advertise<geometry_msgs::Twist>("/nav_vel", 1);

    private_nh.param("max_distance", max_distance_, 0.5);
    private_nh.param("linear_vel", linear_vel_, 0.3);
    private_nh.param("check_frequency", check_frequency_, 10.0);
    private_nh.param("forward_check_distance", forward_check_distance_, 0.6);

    int temp_threshold;
    private_nh.param("max_cost_threshold", temp_threshold,
                     static_cast<int>(costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 10));
    max_cost_threshold_ = static_cast<unsigned char>(temp_threshold);

    double timeout_seconds;
    private_nh.param("timeout", timeout_seconds, 3.0);
    timeout_ = ros::Duration(timeout_seconds);

    // Subscribe to GPS pose for accuracy-aware forward path check
    ros::NodeHandle nh;
    xb_pose_sub_ = nh.subscribe("/xbot_positioning/xb_pose", 5,
                                &BackwardForwardRecovery::onXbPose, this,
                                ros::TransportHints().tcpNoDelay(true));

    initialized_ = true;
  }
  else
  {
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

void BackwardForwardRecovery::onXbPose(const xbot_msgs::AbsolutePose::ConstPtr& msg)
{
  last_xb_pose_ = *msg;
  has_xb_pose_ = true;
}

void BackwardForwardRecovery::runBehavior()
{
  if (!initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  ROS_WARN("BackwardForwardRecovery: Starting recovery");

  // Step 1 — always try backward first
  if (attemptMove(max_distance_, false))
  {
    ROS_INFO("BackwardForwardRecovery: Moved backwards successfully");
  }
  else
  {
    ROS_WARN("BackwardForwardRecovery: Backward move blocked or timed out");
  }

  // Step 2 — only attempt forward if the path ahead is clear
  // Scale the check distance by GPS accuracy: poor GPS → shorter check (less reliable lookahead)
  double check_dist = forward_check_distance_;
  if (has_xb_pose_)
  {
    const double accuracy = last_xb_pose_.position_accuracy;  // meters, lower = better
    // If accuracy is worse than 0.15 m, reduce forward check distance proportionally
    if (accuracy > 0.15)
    {
      check_dist *= std::max(0.3, 0.15 / accuracy);
      ROS_INFO_STREAM("BackwardForwardRecovery: GPS accuracy=" << accuracy
                      << "m → reduced forward check to " << check_dist << "m");
    }
  }

  if (!isForwardPathClear(check_dist))
  {
    ROS_WARN_STREAM("BackwardForwardRecovery: Forward path blocked within " << check_dist
                    << "m — skipping forward attempt");
    return;
  }

  if (attemptMove(max_distance_, true))
  {
    ROS_INFO("BackwardForwardRecovery: Moved forwards successfully");
  }
  else
  {
    ROS_WARN("BackwardForwardRecovery: Forward move blocked or timed out");
  }
}

bool BackwardForwardRecovery::isForwardPathClear(double check_distance)
{
  geometry_msgs::PoseStamped robot_pose;
  if (!local_costmap_->getRobotPose(robot_pose)) return false;

  const double yaw = tf2::getYaw(robot_pose.pose.orientation);
  const double step = local_costmap_->getCostmap()->getResolution();
  const int steps = static_cast<int>(check_distance / step) + 1;

  for (int i = 1; i <= steps; ++i)
  {
    const double wx = robot_pose.pose.position.x + i * step * std::cos(yaw);
    const double wy = robot_pose.pose.position.y + i * step * std::sin(yaw);
    unsigned int mx, my;
    if (!local_costmap_->getCostmap()->worldToMap(wx, wy, mx, my))
      return false;  // outside costmap → treat as blocked
    const unsigned char cost = local_costmap_->getCostmap()->getCost(mx, my);
    if (cost > max_cost_threshold_)
      return false;
  }
  return true;
}

bool BackwardForwardRecovery::attemptMove(double distance, bool forward)
{
  geometry_msgs::PoseStamped start_pose;
  local_costmap_->getRobotPose(start_pose);

  ros::Rate rate(check_frequency_);
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = forward ? linear_vel_ : -linear_vel_;

  double moved_distance = 0.0;
  ros::Time start_time = ros::Time::now();
  while (moved_distance < distance && (ros::Time::now() - start_time) < timeout_)
  {
    geometry_msgs::PoseStamped current_pose;
    local_costmap_->getRobotPose(current_pose);

    moved_distance = std::hypot(
        current_pose.pose.position.x - start_pose.pose.position.x,
        current_pose.pose.position.y - start_pose.pose.position.y);

    if (!isPositionValid(current_pose.pose.position.x, current_pose.pose.position.y))
    {
      ROS_WARN("BackwardForwardRecovery: Cost limit reached after %.2f m", moved_distance);
      cmd_vel.linear.x = 0;
      cmd_vel_pub_.publish(cmd_vel);
      return false;
    }

    cmd_vel_pub_.publish(cmd_vel);
    rate.sleep();
  }

  cmd_vel.linear.x = 0;
  cmd_vel_pub_.publish(cmd_vel);

  if (moved_distance >= distance)
  {
    return true;
  }
  ROS_WARN("BackwardForwardRecovery: %s timed out (%.2f m / %.2f m)",
           forward ? "Forward" : "Backward", moved_distance, distance);
  return false;
}

bool BackwardForwardRecovery::isPositionValid(double x, double y)
{
  unsigned int mx, my;
  if (local_costmap_->getCostmap()->worldToMap(x, y, mx, my))
  {
    unsigned char cost = local_costmap_->getCostmap()->getCost(mx, my);
    return (cost <= max_cost_threshold_);
  }
  return false;
}

}  // namespace ftc_local_planner

PLUGINLIB_EXPORT_CLASS(ftc_local_planner::BackwardForwardRecovery, nav_core::RecoveryBehavior)