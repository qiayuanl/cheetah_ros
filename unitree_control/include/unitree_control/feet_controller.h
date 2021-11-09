//
// Created by qiayuan on 2021/11/9.
//

#pragma once
#include "legs_controller.h"
#include "foot_swing_trajectory.h"

namespace unitree_ros
{
enum FootState
{
  TOUCH,
  SWING
};

struct FootData
{
  FootState state_;
  double phase_, swing_time_;
  Eigen::Vector3d foot_force_estimated;
};

class FeetController : public LegsController
{
public:
  FeetController() = default;
  void updateData(const ros::Time& time, const ros::Duration& period) override;
  void updateCommand(const ros::Time& time, const ros::Duration& period) override;
  FootData& getFootData(LegPrefix leg);
  void setSwing(LegPrefix leg, Eigen::Vector3d final_pos, double swing_time);
  void setTouch(LegPrefix leg, Eigen::Vector3d force);

private:
  FootSwingTrajectory<double> feet_swing_trajectory_[4];
  FootData feet_data_[4];
};

}  // namespace unitree_ros
