//
// Created by qiayuan on 2021/11/15.
//

#pragma once
#include <unitree_control/feet_controller.h>
#include "mpc_solver.h"
#include "gait.h"

namespace unitree_ros
{
class LocomotionController : public FeetController
{
public:
  LocomotionController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void updateData(const ros::Time& time, const ros::Duration& period) override;
  void updateCommand(const ros::Time& time, const ros::Duration& period) override;

protected:
  std::map<std::string, OffsetDurationGaitRos<double>> name2gaits_;
};

}  // namespace unitree_ros
