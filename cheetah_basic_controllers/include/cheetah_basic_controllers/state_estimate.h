//
// Created by qiayuan on 2021/11/15.
//

#pragma once
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_buffer.h>

#include "cpp_types.h"

namespace cheetah_ros
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

class FromTopicStateEstimate : public StateEstimateBase
{
public:
  FromTopicStateEstimate(ros::NodeHandle& nh);
  ~FromTopicStateEstimate() override{};

  void update(RobotState& state) override;

private:
  void callback(const nav_msgs::Odometry::ConstPtr& msg);

  ros::Subscriber sub_;
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_;
};

}  // namespace cheetah_ros
