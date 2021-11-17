//
// Created by qiayuan on 2021/11/15.
//
#include "cheetah_controllers/locomotion.h"
#include <pluginlib/class_list_macros.hpp>

namespace unitree_ros
{
bool LocomotionController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  if (!FeetController::init(robot_hw, controller_nh))
    return false;
  return true;
}

void LocomotionController::updateData(const ros::Time& time, const ros::Duration& period)
{
  ControllerBase::updateData(time, period);
}

}  // namespace unitree_ros

PLUGINLIB_EXPORT_CLASS(unitree_ros::LocomotionController, controller_interface::ControllerBase)
