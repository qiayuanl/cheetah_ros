//
// Created by qiayuan on 2022/1/18.
//

#pragma once
#include "cheetah_mpc_controllers/locomotion.h"

#include <trajectory_msgs/JointTrajectory.h>

namespace cheetah_ros
{
class JumpController : public LocomotionBase
{
public:
  JumpController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  using LocomotionBase::updateData;
  void updateCommand(const ros::Time& time, const ros::Duration& period) override;

private:
  void trajCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg);

  ros::Subscriber traj_sub_;
  realtime_tools::RealtimeBuffer<trajectory_msgs::JointTrajectory> traj_buffer_;
};

}  // namespace cheetah_ros
