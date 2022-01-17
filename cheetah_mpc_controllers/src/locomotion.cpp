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

  double mass = 21.5;
  Matrix3d inertia;
  inertia << 0.050874, 0., 0., 0., 0.64036, 0., 0., 0., 0.6565;

  solver_ = std::make_shared<QpOasesSolver>(mass, -9.81, 0.3, inertia);

  gait_ = name2gaits_.begin()->second;

  // Dynamic reconfigure
  ros::NodeHandle nh_mpc = ros::NodeHandle(controller_nh, "mpc");
  dynamic_srv_ = std::make_shared<dynamic_reconfigure::Server<cheetah_ros::WeightConfig>>(nh_mpc);
  dynamic_reconfigure::Server<cheetah_ros::WeightConfig>::CallbackType cb = [this](auto&& PH1, auto&& PH2) {
    dynamicCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
  };
  dynamic_srv_->setCallback(cb);

  traj_sub_ =
      controller_nh.subscribe<trajectory_msgs::JointTrajectory>("/traj", 1, &LocomotionController::trajCallback, this);
  return true;
}

void LocomotionController::updateData(const ros::Time& time, const ros::Duration& period)
{
  FeetController::updateData(time, period);
}

void LocomotionController::updateCommand(const ros::Time& time, const ros::Duration& period)
{
  gait_->update(time);
  Vec4<int> contact_state_desired = gait_->getContactState();
  Vec4<double> swing_time = gait_->getSwingTime();
  double sign_fr[4] = { 1.0, 1.0, -1.0, -1.0 };
  double sign_lr[4] = { 1.0, -1.0, 1.0, -1.0 };

  int horizon = solver_->getHorizon();
  double dt = solver_->getDt();

  Eigen::VectorXd traj;
  traj.resize(12 * horizon);
  traj.setZero();
  const trajectory_msgs::JointTrajectory traj_msg = *traj_buffer_.readFromRT();
  auto point = traj_msg.points.begin();
  for (int h = 0; h < horizon; ++h)
    while ((traj_msg.header.stamp + point->time_from_start) < time + ros::Duration(horizon * dt))
    {
      int i = 0;
      for (size_t name = 0; name < traj_msg.joint_names.size(); ++name)
      {
        if (traj_msg.joint_names[name] == "roll")
          i = 0;
        else if (traj_msg.joint_names[name] == "pitch")
          i = 1;
        else if (traj_msg.joint_names[name] == "yaw")
          i = 2;
        else if (traj_msg.joint_names[name] == "x")
          i = 3;
        else if (traj_msg.joint_names[name] == "y")
          i = 4;
        else if (traj_msg.joint_names[name] == "z")
          i = 5;
        for (int j = h; j < horizon; ++j)
        {
          traj[12 * j + i] = point->positions[name];
          traj[12 * j + i + 6] = point->velocities[name];
        }
      }
      if (++point == traj_msg.points.end())
        break;
    }

  solver_->solve(time, robot_state_, gait_->getMpcTable(horizon), traj);
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

void LocomotionController::dynamicCallback(WeightConfig& config, uint32_t /*level*/)  // TODO: real time safe
{
  Matrix<double, 13, 1> weight;
  weight << config.ori_roll, config.ori_pitch, config.ori_yaw, config.pos_x, config.pos_y, config.pos_z,
      config.rate_roll, config.rate_pitch, config.rate_yaw, config.vel_x, config.vel_y, config.vel_z, 0.;

  solver_->setup(gait_->getCycle() / static_cast<double>(config.horizon), config.horizon, 666., weight, config.alpha);
  ROS_INFO("[Mpc] Dynamic params update");
}

void LocomotionController::trajCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
  traj_buffer_.writeFromNonRT(*msg);
}

}  // namespace cheetah_ros

PLUGINLIB_EXPORT_CLASS(cheetah_ros::LocomotionController, controller_interface::ControllerBase)
