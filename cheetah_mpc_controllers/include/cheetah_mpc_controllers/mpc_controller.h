//
// Created by qiayuan on 2022/2/6.
//

#pragma once
#include <cheetah_basic_controllers/feet_controller.h>
#include <realtime_tools/realtime_buffer.h>

#include "mpc_solver.h"
#include "gait.h"
#include "cheetah_mpc_controllers/WeightConfig.h"

namespace cheetah_ros
{
class MpcController : public FeetController
{
public:
  MpcController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  using FeetController::updateData;
  void updateCommand(const ros::Time& time, const ros::Duration& period) override;

protected:
  void setTraj(const VectorXd& traj);
  void setGaitTable(const VectorXd& table);

  std::shared_ptr<MpcSolverBase> solver_;
  int horizon_;

private:
  void dynamicCallback(cheetah_ros::WeightConfig& config, uint32_t /*level*/);

  VectorXd gait_table_;
  VectorXd traj_;

  // Dynamic reconfigure
  std::shared_ptr<dynamic_reconfigure::Server<cheetah_ros::WeightConfig>> dynamic_srv_{};
};

}  // namespace cheetah_ros
