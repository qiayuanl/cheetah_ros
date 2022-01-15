//
// Created by qiayuan on 2021/11/15.
//

#pragma once
#include <cheetah_basic_controllers/feet_controller.h>
#include "mpc_solver.h"
#include "gait.h"

namespace cheetah_ros
{
class LocomotionController : public FeetController
{
public:
  LocomotionController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void updateData(const ros::Time& time, const ros::Duration& period) override;
  void updateCommand(const ros::Time& time, const ros::Duration& period) override;

protected:
  std::map<std::string, OffsetDurationGaitRos<double>::Ptr> name2gaits_;
  std::shared_ptr<MpcSolverBase> solver_;
  const int horizon_ = 10;
};

}  // namespace cheetah_ros
