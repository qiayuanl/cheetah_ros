//
// Created by qiayuan on 2021/11/15.
//
#include "cheetah_mpc_controllers/mpc_controller.h"

#include <pluginlib/class_list_macros.hpp>

namespace cheetah_ros
{
bool MpcController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  if (!FeetController::init(robot_hw, controller_nh))
    return false;

  // TODO: Add interface
  double mass = 21.5;
  Matrix3d inertia;
  inertia << 0.050874, 0., 0., 0., 0.64036, 0., 0., 0., 0.6565;

  solver_ = std::make_shared<QpOasesSolver>(mass, -9.81, 0.6, inertia);

  // Dynamic reconfigure
  ros::NodeHandle nh_mpc = ros::NodeHandle(controller_nh, "mpc");
  dynamic_srv_ = std::make_shared<dynamic_reconfigure::Server<cheetah_ros::WeightConfig>>(nh_mpc);
  dynamic_reconfigure::Server<cheetah_ros::WeightConfig>::CallbackType cb = [this](auto&& PH1, auto&& PH2) {
    dynamicCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
  };
  dynamic_srv_->setCallback(cb);

  traj_.resize(12 * horizon_);
  traj_.setZero();

  return true;
}

void MpcController::updateCommand(const ros::Time& time, const ros::Duration& period)
{
  solver_->solve(time, robot_state_, gait_table_, traj_);
  std::vector<Vec3<double>> solution = solver_->getSolution();
  for (int i = 0; i < 4; ++i)
    if (gait_table_[i] == 1)
      setStand(LegPrefix(i), solution[i]);
  ROS_ERROR_STREAM(gait_table_);
  FeetController::updateCommand(time, period);
}

void MpcController::setTraj(const VectorXd& traj)
{
  traj_ = traj;
}

void MpcController::setGaitTable(const VectorXd& table)
{
  gait_table_ = table;
}

void MpcController::dynamicCallback(WeightConfig& config, uint32_t /*level*/)  // TODO: real time safe
{
  Matrix<double, 13, 1> weight;
  weight << config.ori_roll, config.ori_pitch, config.ori_yaw, config.pos_x, config.pos_y, config.pos_z,
      config.rate_roll, config.rate_pitch, config.rate_yaw, config.vel_x, config.vel_y, config.vel_z, 0.;
  horizon_ = config.horizon;
  solver_->setup(config.dt, config.horizon, 200., weight, config.alpha);
  ROS_INFO("[Mpc] Dynamic params update");
}

}  // namespace cheetah_ros
