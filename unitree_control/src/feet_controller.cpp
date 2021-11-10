//
// Created by qiayuan on 2021/11/9.
//
#include "unitree_control/feet_controller.h"

#include <pluginlib/class_list_macros.hpp>

namespace unitree_ros
{
void FeetController::updateData(const ros::Time& time, const ros::Duration& period)
{
  LegsController::updateData(time, period);
}

void FeetController::updateCommand(const ros::Time& time, const ros::Duration& period)
{
  for (int leg = 0; leg < 4; ++leg)
  {
    if (feet_data_[leg].state_ == SWING)
    {
      feet_data_[leg].phase_ += period.toSec() / feet_data_[leg].swing_time_;
      if (feet_data_[leg].phase_ > 1.)
        feet_data_[leg].phase_ = 1.;
      feet_swing_trajectory_[leg].computeSwingTrajectoryBezier(feet_data_[leg].phase_, feet_data_[leg].swing_time_);
      LegsController::Command leg_cmd;
      leg_cmd.foot_pos_des_ = feet_swing_trajectory_[leg].getPosition();
      leg_cmd.foot_vel_des_ = feet_swing_trajectory_[leg].getVelocity();
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

FootData& FeetController::getFootData(LegPrefix leg)
{
  return feet_data_[leg];
}

void FeetController::setSwing(LegPrefix leg, const Eigen::Vector3d& final_pos, double swing_time)
{
  feet_data_[leg].state_ = SWING;
  feet_data_[leg].phase_ = 0.;
  feet_data_[leg].swing_time_ = swing_time;
  feet_swing_trajectory_[leg].setHeight(0.1);
  feet_swing_trajectory_[leg].setInitialPosition(getLegState(leg).foot_pos_);
  feet_swing_trajectory_[leg].setFinalPosition(final_pos);
}

void FeetController::setTouch(LegPrefix leg, const Eigen::Vector3d& force)
{
}

}  // namespace unitree_ros

PLUGINLIB_EXPORT_CLASS(unitree_ros::FeetController, controller_interface::ControllerBase)
