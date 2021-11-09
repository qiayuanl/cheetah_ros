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
  FR,
  RL,
  RR
};

const static std::string LEG_PREFIX[4] = { "FL", "FR", "RL", "RR" };

struct LegJoint
{
  HybridJointHandle joints_[3];
};

struct LegData
{
  Eigen::Vector3d foot_pos_, foot_vel_;
};

struct LegCommand
{
  Eigen::Vector3d foot_pos_des_, foot_vel_des_, ff_cartesian_;
  Eigen::Matrix3d kp_cartesian_, kd_cartesian_;
};

class LegsController
  : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::JointStateInterface>
{
public:
  LegsController() = default;
  LegsController(std::shared_ptr<pinocchio::Model> model, std::shared_ptr<pinocchio::Data> data);
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void updateData(const ros::Time& time, const ros::Duration& period);
  void updateCommand(const ros::Time& time, const ros::Duration& period);
  LegJoint& getLegJoint(LegPrefix leg);
  const LegData& getData(LegPrefix leg);
  void setCommand(LegPrefix leg, const LegCommand& command);

private:
  std::shared_ptr<pinocchio::Model> pin_model_;
  std::shared_ptr<pinocchio::Data> pin_data_;
  LegJoint leg_joints_[4];
  LegData datas_[4];
  LegCommand commands_[4];
};

};  // namespace unitree_ros
