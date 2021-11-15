//
// Created by qiayuan on 2021/11/15.
//
#include "cheetah_controllers/state_estimate.h"

namespace unitree_ros
{
GroundTruthStateEstimate::GroundTruthStateEstimate(ros::NodeHandle& nh) : StateEstimateBase(nh)
{
  sub_ = nh.subscribe<nav_msgs::Odometry>("/ground_truth/state", 100, &GroundTruthStateEstimate::callback, this);
}

void GroundTruthStateEstimate::callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  buffer_.writeFromNonRT(*msg);
}

const StateEstimateBase::State& GroundTruthStateEstimate::getState()
{
  nav_msgs::Odometry odom = *buffer_.readFromRT();
  state_.pos_ << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;
  state_.quat_ << odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
      odom.pose.pose.orientation.w;
  state_.linear_vel_ << odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z;
  state_.angular_vel_ << odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z;

  return StateEstimateBase::getState();
}

}  // namespace unitree_ros
