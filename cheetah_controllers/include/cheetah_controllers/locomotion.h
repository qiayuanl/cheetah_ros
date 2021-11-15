//
// Created by qiayuan on 2021/11/15.
//

#pragma once
#include "state_estimate.h"

#include <unitree_control/feet_controller.h>

namespace unitree_ros
{
class LocomotionController : public FeetController
{
public:
  LocomotionController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void updateData(const ros::Time& time, const ros::Duration& period) override;

protected:
  std::shared_ptr<StateEstimateBase> state_estimate_;
};

}  // namespace unitree_ros
