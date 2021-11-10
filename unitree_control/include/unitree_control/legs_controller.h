//
// Created by qiayuan on 2021/11/6.
//

#pragma once
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <urdf_parser/urdf_parser.h>

#include <controller_interface/multi_interface_controller.h>
#include <unitree_common/hardware_interface/hybrid_joint_interface.h>

#include <unitree_msgs/LegsCmd.h>
#include <unitree_msgs/LegsState.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

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

class LegsController
  : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::JointStateInterface>
{
public:
  struct Joints
  {
    HybridJointHandle joints_[3];
  };
  struct State
  {
    Eigen::Vector3d foot_pos_, foot_vel_;
  };
  struct Command
  {
    ros::Time stamp_;
    Eigen::Vector3d foot_pos_des_, foot_vel_des_, ff_cartesian_;
    Eigen::Matrix3d kp_cartesian_, kd_cartesian_;
  };

  LegsController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  virtual void updateData(const ros::Time& time, const ros::Duration& period);
  virtual void updateCommand(const ros::Time& time, const ros::Duration& period);
  Joints& getLegJoints(LegPrefix leg);
  const State& getLegState(LegPrefix leg);
  void setLegCmd(LegPrefix leg, const Command& command);

protected:
  void publishState(const ros::Time& time, const ros::Duration& period);

  std::shared_ptr<pinocchio::Model> pin_model_;
  std::shared_ptr<pinocchio::Data> pin_data_;

private:
  void legsCmdCallback(const unitree_msgs::LegsCmd::ConstPtr& msg);

  Joints leg_joints_[4];
  State states_[4];
  Command commands_[4];

  ros::Subscriber legs_cmd_sub_;
  realtime_tools::RealtimeBuffer<unitree_msgs::LegsCmd> legs_cmd_buffer_;
  std::shared_ptr<realtime_tools::RealtimePublisher<unitree_msgs::LegsState> > state_pub_;
  ros::Time last_publish_;
};

};  // namespace unitree_ros
