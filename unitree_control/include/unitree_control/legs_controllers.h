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
enum LegPrefix
{
  FL,
  FF,
  RL,
  RR
};

const static std::string LEG_PREFIX[4] = { "FL", "FF", "RL", "RR" };

struct LegData
{
  HybridJointHandle joints_[3];
  Eigen::Vector3d foot_pos_, foot_vel_;
};

class LegsController : public controller_interface::MultiInterfaceController<HybridJointInterface>
{
public:
  LegsController() = default;
  LegsController(std::shared_ptr<pinocchio::Model> model, std::shared_ptr<pinocchio::Data> data);
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  void updateData();
  std::shared_ptr<pinocchio::Model> pin_model_;
  std::shared_ptr<pinocchio::Data> pin_data_;
  LegData datas_[4];
  ros::Time last_publish_;
};

};  // namespace unitree_ros
