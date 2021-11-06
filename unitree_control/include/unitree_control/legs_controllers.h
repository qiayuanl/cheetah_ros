//
// Created by qiayuan on 2021/11/6.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <unitree_common/hardware_interface/hybrid_joint_interface.h>
#include <pinocchio/multibody/model.hpp>
#include <utility>

namespace unitree_ros
{
class LegsController : public controller_interface::MultiInterfaceController<HybridJointInterface>
{
public:
  LegsController() = default;
  LegsController(std::shared_ptr<pinocchio::Model> model, std::shared_ptr<pinocchio::Data> data);
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  std::shared_ptr<pinocchio::Model> model_;
  std::shared_ptr<pinocchio::Data> data_;
  std::vector<HybridJointHandle> joint_handles_;
};

};  // namespace unitree_ros
