//
// Created by qiayuan on 2021/11/9.
//

#pragma once
#include "legs_controller.h"
#include "foot_swing_trajectory.h"

namespace unitree_ros
{
struct FootCommand
{
  double time;
  Eigen::Vector3d pos_final;
};

class FeetController : public LegsController
{
public:
  FeetController() = default;
  FeetController(std::shared_ptr<pinocchio::Model> model, std::shared_ptr<pinocchio::Data> data);
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void updateData(const ros::Time& time, const ros::Duration& period);
  void updateCommand(const ros::Time& time, const ros::Duration& period);

private:
  FootSwingTrajectory<double> feet_swing_trajectory_[4];
  FootCommand feet_command_[4];
};

}  // namespace unitree_ros
