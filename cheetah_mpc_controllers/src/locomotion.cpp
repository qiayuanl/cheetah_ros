//
// Created by qiayuan on 2021/11/15.
//
#include "cheetah_mpc_controllers/locomotion.h"

#include <pluginlib/class_list_macros.hpp>

namespace cheetah_ros
{
bool LocomotionController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  if (!FeetController::init(robot_hw, controller_nh))
    return false;
  XmlRpc::XmlRpcValue gaits_params;
  controller_nh.getParam("gaits", gaits_params);
  ROS_ASSERT(gaits_params.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for (auto gait_params : gaits_params)
    name2gaits_.insert(
        std::make_pair(gait_params.first.c_str(), std::make_shared<OffsetDurationGaitRos<double>>(gait_params.second)));

  double mass = 11.041;
  Matrix3d inertia;
  inertia << 0.050874, 0., 0., 0., 0.64036, 0., 0., 0., 0.6565;

  solver_ = std::make_shared<QpOasesSolver>(mass, -9.81, 0.3, inertia);
  Matrix<double, 13, 1> weight;
  weight << 0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2, 0.;

  solver_->setup(0.03, horizon_, 666., weight, 1e-6);

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

  Eigen::VectorXd traj;
  traj.resize(12 * horizon_);
  traj.setZero();
  for (int i = 0; i < horizon_; ++i)
  {
    traj[12 * i + 5] = 0.35;
  }

  solver_->solve(time, robot_state_, gait->getMpcTable(horizon_), traj);
  std::vector<Vec3<double>> solution = solver_->getSolution();

  for (int j = 0; j < 4; ++j)
  {
    LegPrefix leg = LegPrefix(j);
    if (contact_state_desired[j] == 1)
      setStand(leg, solution[j]);

    else if (contact_state_desired[j] == 0 && getFootState(leg) == STAND)
    {
      Eigen::Vector3d pos;
      pos << sign_fr[j] * 0.25, sign_lr[j] * 0.2, 0.;
      setSwing(leg, pos, 0.1, swing_time[j]);
    }
  }

  FeetController::updateCommand(time, period);
}

}  // namespace cheetah_ros

PLUGINLIB_EXPORT_CLASS(cheetah_ros::LocomotionController, controller_interface::ControllerBase)
