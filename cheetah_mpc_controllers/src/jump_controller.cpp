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

  int horizon = solver_->getHorizon();
  double dt = solver_->getDt();

  Eigen::VectorXd traj;
  traj.resize(12 * horizon);
  traj.setZero();
  auto point = traj_msg.points.begin();
  for (int h = 0; h < horizon && point != traj_msg.points.end(); ++h)
    while (time > traj_msg.header.stamp &&
           (traj_msg.header.stamp + point->time_from_start) < time + ros::Duration(horizon * dt))
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

  setTraj(traj);
  LocomotionBase::updateCommand(time, period);
}

void JumpController::trajCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
  traj_buffer_.writeFromNonRT(*msg);
}

}  // namespace cheetah_ros

PLUGINLIB_EXPORT_CLASS(cheetah_ros::JumpController, controller_interface::ControllerBase)
