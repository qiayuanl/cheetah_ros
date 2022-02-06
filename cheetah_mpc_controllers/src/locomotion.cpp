//
// Created by qiayuan on 2021/11/15.
//
#include "cheetah_mpc_controllers/locomotion.h"

#include <pluginlib/class_list_macros.hpp>

namespace cheetah_ros
{
bool LocomotionBase::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  if (!MpcController::init(robot_hw, controller_nh))
    return false;
  XmlRpc::XmlRpcValue gaits_params;
  controller_nh.getParam("gaits", gaits_params);
  ROS_ASSERT(gaits_params.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for (auto gait_params : gaits_params)
    name2gaits_.insert(
        std::make_pair(gait_params.first.c_str(), std::make_shared<OffsetDurationGaitRos<double>>(gait_params.second)));
  gait_ = name2gaits_.begin()->second;

  return true;
}

void LocomotionBase::updateCommand(const ros::Time& time, const ros::Duration& period)
{
  Eigen::VectorXd traj;
  traj.resize(12 * horizon_);
  traj.setZero();
  for (int i = 0; i < horizon_; ++i)
    traj[12 * i + 5] = 0.1;
  setTraj(traj);

  gait_->update(time);
  Eigen::VectorXd table = gait_->getMpcTable(solver_->getHorizon());
  setGaitTable(table);
  Vec4<double> swing_time = gait_->getSwingTime();
  double sign_fr[4] = { 1.0, 1.0, -1.0, -1.0 };
  double sign_lr[4] = { 1.0, -1.0, 1.0, -1.0 };

  for (int i = 0; i < 4; ++i)
  {
    LegPrefix leg = LegPrefix(i);
    if (table[i] == 0 && getFootState(leg) == STAND)
    {
      Eigen::Vector3d pos;
      pos << sign_fr[i] * 0.25, sign_lr[i] * 0.15, 0.;  // TODO footstep
      setSwing(leg, pos, 0.05, swing_time[i]);
    }
  }

  MpcController::updateCommand(time, period);
}

}  // namespace cheetah_ros

PLUGINLIB_EXPORT_CLASS(cheetah_ros::LocomotionBase, controller_interface::ControllerBase)
