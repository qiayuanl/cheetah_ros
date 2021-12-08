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
  XmlRpc::XmlRpcValue gaits_params;
  controller_nh.getParam("gaits", gaits_params);
  ROS_ASSERT(gaits_params.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for (auto gait_params : gaits_params)
    name2gaits_.insert(std::make_pair(gait_params.first.c_str(),
                                      std::make_shared<OffsetDurationGaitRos<double>>(gait_params.second, 10)));
  return true;
}

void LocomotionController::updateData(const ros::Time& time, const ros::Duration& period)
{
  FeetController::updateData(time, period);
}

void LocomotionController::updateCommand(const ros::Time& time, const ros::Duration& period)
{
  auto gait = name2gaits_.begin()->second;
  gait->update(time);
  Vec4<int> contact_state_desired = gait->getContactState();
  Vec4<double> swing_time = gait->getSwingTime();
  double sign_fr[4] = { 1.0, 1.0, -1.0, -1.0 };
  double sign_lr[4] = { 1.0, -1.0, 1.0, -1.0 };

  for (int j = 0; j < 4; ++j)
  {
    LegPrefix leg = LegPrefix(j);
    if (contact_state_desired[j] == 1)
    {
      Eigen::Vector3d force;
      force.setZero();
      setStand(leg, force);
    }
    else if (contact_state_desired[j] == 0 && getFootState(leg) == STAND)
    {
      Eigen::Vector3d pos;
      pos << sign_fr[j] * 0.2, sign_lr[j] * 0.2, 0.75;
      setSwing(leg, pos, 0.1, swing_time[j]);
    }
  }
  FeetController::updateCommand(time, period);
}

}  // namespace unitree_ros

PLUGINLIB_EXPORT_CLASS(unitree_ros::LocomotionController, controller_interface::ControllerBase)
