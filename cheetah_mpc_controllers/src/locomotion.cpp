//
// Created by qiayuan on 2021/11/15.
//
#include "cheetah_mpc_controllers/locomotion.h"

namespace cheetah_ros
{
bool LocomotionBase::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  if (!FeetController::init(robot_hw, controller_nh))
    return false;
  XmlRpc::XmlRpcValue gaits_params;
  controller_nh.getParam("gaits", gaits_params);
  ROS_ASSERT(gaits_params.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for (auto gait_params : gaits_params)
    name2gaits_.insert(
        std::make_pair(gait_params.first.c_str(), std::make_shared<OffsetDurationGaitRos<double>>(gait_params.second)));

  double mass = 21.5;
  Matrix3d inertia;
  inertia << 0.050874, 0., 0., 0., 0.64036, 0., 0., 0., 0.6565;

  solver_ = std::make_shared<QpOasesSolver>(mass, -9.81, 0.3, inertia);

  gait_ = name2gaits_.begin()->second;
  traj_.setZero();

  // Dynamic reconfigure
  ros::NodeHandle nh_mpc = ros::NodeHandle(controller_nh, "mpc");
  dynamic_srv_ = std::make_shared<dynamic_reconfigure::Server<cheetah_ros::WeightConfig>>(nh_mpc);
  dynamic_reconfigure::Server<cheetah_ros::WeightConfig>::CallbackType cb = [this](auto&& PH1, auto&& PH2) {
    dynamicCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
  };
  dynamic_srv_->setCallback(cb);

  return true;
}

void LocomotionBase::updateCommand(const ros::Time& time, const ros::Duration& period)
{
  gait_->update(time);
  Vec4<int> contact_state_desired = gait_->getContactState();
  Vec4<double> swing_time = gait_->getSwingTime();
  double sign_fr[4] = { 1.0, 1.0, -1.0, -1.0 };
  double sign_lr[4] = { 1.0, -1.0, 1.0, -1.0 };

  solver_->solve(time, robot_state_, gait_->getMpcTable(solver_->getHorizon()), traj_);
  std::vector<Vec3<double>> solution = solver_->getSolution();

  for (int j = 0; j < 4; ++j)
  {
    LegPrefix leg = LegPrefix(j);
    if (contact_state_desired[j] == 1)
      setStand(leg, solution[j]);

    else if (contact_state_desired[j] == 0 && getFootState(leg) == STAND)
    {
      Eigen::Vector3d pos;
      pos << sign_fr[j] * 0.25, sign_lr[j] * 0.15, 0.0265;
      setSwing(leg, pos, 0.15, swing_time[j]);
    }
  }
  FeetController::updateCommand(time, period);
}

void LocomotionBase::setTraj(const VectorXd& traj)
{
  traj_ = traj;
}

void LocomotionBase::dynamicCallback(WeightConfig& config, uint32_t /*level*/)  // TODO: real time safe
{
  Matrix<double, 13, 1> weight;
  weight << config.ori_roll, config.ori_pitch, config.ori_yaw, config.pos_x, config.pos_y, config.pos_z,
      config.rate_roll, config.rate_pitch, config.rate_yaw, config.vel_x, config.vel_y, config.vel_z, 0.;

  solver_->setup(gait_->getCycle() / static_cast<double>(config.horizon), config.horizon, 666., weight, config.alpha);
  ROS_INFO("[Mpc] Dynamic params update");
}

}  // namespace cheetah_ros
