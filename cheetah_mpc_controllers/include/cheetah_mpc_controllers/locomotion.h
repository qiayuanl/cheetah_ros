//
// Created by qiayuan on 2021/11/15.
//

#pragma once
#include <cheetah_basic_controllers/feet_controller.h>
#include "mpc_solver.h"
#include "gait.h"
#include "cheetah_mpc_controllers/WeightConfig.h"

namespace cheetah_ros
{
class LocomotionController : public FeetController
{
public:
  LocomotionController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void updateData(const ros::Time& time, const ros::Duration& period) override;
  void updateCommand(const ros::Time& time, const ros::Duration& period) override;

private:
  void dynamicCallback(cheetah_ros::WeightConfig& config, uint32_t /*level*/);

  std::map<std::string, OffsetDurationGaitRos<double>::Ptr> name2gaits_;
  OffsetDurationGaitRos<double>::Ptr gait_;
  std::shared_ptr<MpcSolverBase> solver_;
  // Dynamic reconfigure
  std::shared_ptr<dynamic_reconfigure::Server<cheetah_ros::WeightConfig>> dynamic_srv_{};
};

}  // namespace cheetah_ros
