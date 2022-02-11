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

  XmlRpc::XmlRpcValue mpc_params;
  cheetah_ros::WeightConfig config;
  controller_nh.getParam("mpc", mpc_params);
  ROS_ASSERT(mpc_params.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  if (mpc_params.hasMember("weight"))
  {
    if (mpc_params["weight"].getType() == XmlRpc::XmlRpcValue::TypeArray)
      if (mpc_params["weight"].size() == 12)
      {
        config.ori_roll = xmlRpcGetDouble(mpc_params["weight"], 0);
        config.ori_pitch = xmlRpcGetDouble(mpc_params["weight"], 1);
        config.ori_yaw = xmlRpcGetDouble(mpc_params["weight"], 2);
        config.pos_x = xmlRpcGetDouble(mpc_params["weight"], 3);
        config.pos_y = xmlRpcGetDouble(mpc_params["weight"], 4);
        config.pos_z = xmlRpcGetDouble(mpc_params["weight"], 5);
        config.rate_roll = xmlRpcGetDouble(mpc_params["weight"], 6);
        config.rate_pitch = xmlRpcGetDouble(mpc_params["weight"], 7);
        config.rate_yaw = xmlRpcGetDouble(mpc_params["weight"], 8);
        config.vel_x = xmlRpcGetDouble(mpc_params["weight"], 9);
        config.vel_y = xmlRpcGetDouble(mpc_params["weight"], 10);
        config.vel_z = xmlRpcGetDouble(mpc_params["weight"], 11);
        config.alpha = xmlRpcGetDouble(mpc_params["alpha"]);
        config.horizon = mpc_params["horizon"];
        config.dt = xmlRpcGetDouble(mpc_params["dt"]);
        weight_buffer_.initRT(config);
        dynamic_initialized_ = false;
      }
  }
  else
    dynamic_initialized_ = true;

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
  if (!dynamic_initialized_)
  {
    ROS_INFO("[Mpc] Dynamic params are set by rosparams at initialization");

    dynamic_initialized_ = true;
    cheetah_ros::WeightConfig init_config = *weight_buffer_.readFromNonRT();
    config.ori_roll = init_config.ori_roll;
    config.ori_pitch = init_config.ori_pitch;
    config.ori_yaw = init_config.ori_yaw;
    config.pos_x = init_config.pos_x;
    config.pos_y = init_config.pos_y;
    config.pos_z = init_config.pos_z;
    config.rate_roll = init_config.rate_roll;
    config.rate_pitch = init_config.rate_pitch;
    config.rate_yaw = init_config.rate_yaw;
    config.vel_x = init_config.vel_x;
    config.vel_y = init_config.vel_y;
    config.vel_z = init_config.vel_z;
    config.alpha = init_config.alpha;
    config.horizon = init_config.horizon;
    config.dt = init_config.dt;
  }

  Matrix<double, 13, 1> weight;
  weight << config.ori_roll, config.ori_pitch, config.ori_yaw, config.pos_x, config.pos_y, config.pos_z,
      config.rate_roll, config.rate_pitch, config.rate_yaw, config.vel_x, config.vel_y, config.vel_z, 0.;
  horizon_ = config.horizon;
  solver_->setup(config.dt, config.horizon, 200., weight, config.alpha, 1.0);
  ROS_INFO("[Mpc] Dynamic params update");
}

}  // namespace cheetah_ros
