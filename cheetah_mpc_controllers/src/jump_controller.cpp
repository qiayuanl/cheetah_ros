//
// Created by qiayuan on 2022/1/18.
//
#include "cheetah_mpc_controllers/jump_controller.h"

#include <pluginlib/class_list_macros.hpp>

namespace cheetah_ros
{
bool JumpController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  bool ret = LocomotionBase::init(robot_hw, controller_nh);
  traj_sub_ =
      controller_nh.subscribe<trajectory_msgs::JointTrajectory>("/traj", 1, &JumpController::trajCallback, this);
  return ret;
}

void JumpController::updateCommand(const ros::Time& time, const ros::Duration& period)
{
  const trajectory_msgs::JointTrajectory traj_msg = *traj_buffer_.readFromRT();
  if (time < traj_msg.header.stamp ||
      time > traj_msg.header.stamp + traj_msg.points.back().time_from_start + ros::Duration(solver_->getDt()))
  {
    double sign_fr[4] = { 1.0, 1.0, -1.0, -1.0 };
    double sign_lr[4] = { 1.0, -1.0, 1.0, -1.0 };

    for (int i = 0; i < 4; ++i)
    {
      Eigen::Vector3d pos;
      pos << sign_fr[i] * 0.25, sign_lr[i] * 0.15, -0.05;
      pos += robot_state_.pos_;
      LegCmd leg_cmd;
      leg_cmd.foot_pos_des_ = pos;
      leg_cmd.foot_vel_des_.setZero();
      leg_cmd.kp_cartesian_ << 800., 0, 0, 0, 800., 0, 0, 0, 800.;
      leg_cmd.kd_cartesian_ << 10., 0, 0, 0, 10., 0, 0, 0, 10.;
      setLegCmd(LegPrefix(i), leg_cmd);
    }

    solver_->setHorizon(horizon_);
    ControllerBase::updateCommand(time, period);
    return;
  }

  double dt = solver_->getDt();

  Eigen::VectorXd traj;
  traj.resize(12 * horizon_);
  traj.setZero();
  for (int h = 0; h < horizon_; ++h)
  {
    auto point = traj_msg.points.begin() + 1;
    while (point != traj_msg.points.end())
    {
      if (time + ros::Duration(h * dt) < traj_msg.header.stamp + point->time_from_start &&
          time + ros::Duration(h * dt) >= traj_msg.header.stamp + (point - 1)->time_from_start)
        break;
      ++point;
    }
    if (point == traj_msg.points.end())
    {
      if (h < horizon_ - 1)
        solver_->setHorizon(h + 1);
      break;
    }
    for (size_t name = 0; name < traj_msg.joint_names.size(); ++name)
    {
      int i = 0;
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
      traj[12 * h + i] = point->positions[name];
      traj[12 * h + i + 6] = point->velocities[name];
    }
  }

  setTraj(traj);
  LocomotionBase::updateCommand(time, period);
}

void JumpController::trajCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
  traj_buffer_.writeFromNonRT(*msg);
}

}  // namespace cheetah_ros

PLUGINLIB_EXPORT_CLASS(cheetah_ros::JumpController, controller_interface::ControllerBase)
