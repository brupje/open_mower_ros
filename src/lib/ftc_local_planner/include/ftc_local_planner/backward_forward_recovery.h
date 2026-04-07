#ifndef BACKWARD_FORWARD_RECOVERY_H
#define BACKWARD_FORWARD_RECOVERY_H

#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <base_local_planner/costmap_model.h>
#include <geometry_msgs/Twist.h>
#include <xbot_msgs/AbsolutePose.h>
#include <string>

namespace ftc_local_planner
{

class BackwardForwardRecovery : public nav_core::RecoveryBehavior
{
public:
  BackwardForwardRecovery();
  void initialize(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* global_costmap,
                  costmap_2d::Costmap2DROS* local_costmap);
  void runBehavior();

private:
  bool attemptMove(double distance, bool forward);
  bool isPositionValid(double x, double y);
  bool isForwardPathClear(double check_distance);
  void onXbPose(const xbot_msgs::AbsolutePose::ConstPtr& msg);

  std::string name_;
  bool initialized_;
  tf2_ros::Buffer* tf_;
  costmap_2d::Costmap2DROS* global_costmap_, *local_costmap_;
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber xb_pose_sub_;
  xbot_msgs::AbsolutePose last_xb_pose_;
  bool has_xb_pose_{false};

  double max_distance_;
  double linear_vel_;
  double check_frequency_;
  unsigned char max_cost_threshold_;
  ros::Duration timeout_;
  double forward_check_distance_;   // how far ahead to check costmap before forward attempt
};

}  // namespace ftc_local_planner

#endif  // BACKWARD_FORWARD_RECOVERY_H