//
// Created by qiayuan on 2021/11/15.
//
#include "cheetah_basic_controllers/state_estimate.h"

namespace cheetah_ros
{
GroundTruth::GroundTruth(ros::NodeHandle& nh) : StateEstimateBase(nh)
{
  sub_ = nh.subscribe<nav_msgs::Odometry>("/ground_truth/state", 100, &GroundTruth::callback, this);
}

void GroundTruth::callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  buffer_.writeFromNonRT(*msg);
}

void GroundTruth::update(RobotState& state)
{
  nav_msgs::Odometry odom = *buffer_.readFromRT();
  state.pos_ << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;
  state.quat_.coeffs() << odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
      odom.pose.pose.orientation.w;
  state.linear_vel_ << odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z;
  state.angular_vel_ << odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z;
}

}  // namespace cheetah_ros
