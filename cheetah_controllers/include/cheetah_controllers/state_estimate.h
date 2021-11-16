//
// Created by qiayuan on 2021/11/15.
//

#pragma once
#include <unitree_control/legs_controller.h>
#include <nav_msgs/Odometry.h>

namespace unitree_ros
{
class StateEstimateBase
{
public:
  using State = LegsController::BaseState;
  StateEstimateBase(ros::NodeHandle& nh)
  {
  }
  virtual ~StateEstimateBase(){};
  virtual const State& getState()
  {
    return state_;
  }

protected:
  State state_;
};

class GroundTruth : public StateEstimateBase
{
public:
  GroundTruth(ros::NodeHandle& nh);
  ~GroundTruth() override{};

  const State& getState() override;

private:
  void callback(const nav_msgs::Odometry::ConstPtr& msg);

  ros::Subscriber sub_;
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_;
};

}  // namespace unitree_ros
