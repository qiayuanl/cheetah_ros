//
// Created by qiayuan on 2021/11/9.
//
#include "unitree_control/feet_controller.h"

#include <pluginlib/class_list_macros.hpp>

namespace unitree_ros
{
bool FeetController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  if (!LegsController::init(robot_hw, controller_nh))
    return false;

  // ROS Topic
  feet_cmd_sub_ =
      controller_nh.subscribe<unitree_msgs::FeetCmd>("/cmd_feet", 1, &FeetController::feetCmdCallback, this);
  return true;
}

void FeetController::updateCommand(const ros::Time& time, const ros::Duration& period)
{
  // Update Command from ROS topic interface.
  unitree_msgs::FeetCmd cmd_msgs = *feet_cmd_buffer_.readFromRT();
  for (size_t j = 0; j < cmd_msgs.leg_prefix.size(); ++j)
  {
    State& state = states_[cmd_msgs.leg_prefix[j].prefix];
    if (cmd_msgs.header.stamp > state.take_off_time_)
    {
      state.take_off_time_ = cmd_msgs.header.stamp;
      Eigen::Vector3d pos_final(cmd_msgs.pos_final[j].x, cmd_msgs.pos_final[j].y, cmd_msgs.pos_final[j].z);
      if (cmd_msgs.touch_state[j] == cmd_msgs.SWING)
        setSwing(LegPrefix(cmd_msgs.leg_prefix[j].prefix), pos_final, cmd_msgs.swing_time[j]);
    }
  }

  for (int leg = 0; leg < 4; ++leg)
  {
    if (states_[leg].touch_state_ == SWING)
    {
      states_[leg].phase_ += period.toSec() / states_[leg].swing_time_;
      if (states_[leg].phase_ > 1.)
        states_[leg].phase_ = 1.;
      swing_trajectory_[leg].computeSwingTrajectoryBezier(states_[leg].phase_, states_[leg].swing_time_);
      LegsController::Command leg_cmd;
      leg_cmd.stamp_ = time;
      leg_cmd.foot_pos_des_ = swing_trajectory_[leg].getPosition();
      leg_cmd.foot_vel_des_ = swing_trajectory_[leg].getVelocity();
      leg_cmd.kp_cartesian_ << 700, 0, 0, 0, 700, 0, 0, 0, 150;
      leg_cmd.kd_cartesian_ << 7, 0, 0, 0, 7, 0, 0, 0, 7;
      setLegCmd(LegPrefix(leg), leg_cmd);
    }
    else
    {
      LegsController::Command leg_cmd{};
      leg_cmd.kp_cartesian_ << 0, 0, 0, 0, 0, 0, 0, 0, 0;
      leg_cmd.kd_cartesian_ << 0, 0, 0, 0, 0, 0, 0, 0, 0;
      leg_cmd.ff_cartesian_ << 0, 0, 0;
      setLegCmd(LegPrefix(leg), leg_cmd);
    }
  }
  LegsController::updateCommand(time, period);
}

FeetController::State& FeetController::getFootState(LegPrefix leg)
{
  return states_[leg];
}

void FeetController::setSwing(LegPrefix leg, const Eigen::Vector3d& final_pos, double swing_time)
{
  states_[leg].touch_state_ = SWING;
  states_[leg].phase_ = 0.;
  states_[leg].swing_time_ = swing_time;
  swing_trajectory_[leg].setHeight(0.1);
  swing_trajectory_[leg].setInitialPosition(getLegState(leg).foot_pos_);
  swing_trajectory_[leg].setFinalPosition(final_pos);
}

void FeetController::setTouch(LegPrefix leg, const Eigen::Vector3d& force)
{
}

void FeetController::feetCmdCallback(const unitree_msgs::FeetCmd::ConstPtr& msg)
{
  feet_cmd_buffer_.writeFromNonRT(*msg);
}

}  // namespace unitree_ros

PLUGINLIB_EXPORT_CLASS(unitree_ros::FeetController, controller_interface::ControllerBase)
