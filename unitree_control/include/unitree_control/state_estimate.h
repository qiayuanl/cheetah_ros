//
// Created by qiayuan on 2021/11/15.
//

#pragma once
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_buffer.h>

#include "cpp_types.h"

namespace unitree_ros
{
class StateEstimateBase
{
public:
  StateEstimateBase(ros::NodeHandle& nh)
  {
  }
  virtual ~StateEstimateBase(){};
  virtual void update(RobotState& state){};
};

class GroundTruth : public StateEstimateBase
{
public:
  GroundTruth(ros::NodeHandle& nh);
  ~GroundTruth() override{};

  void update(RobotState& state) override;

private:
  void callback(const nav_msgs::Odometry::ConstPtr& msg);

  ros::Subscriber sub_;
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_;
};

}  // namespace unitree_ros
