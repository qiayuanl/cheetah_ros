//
// Created by qiayuan on 2021/11/6.
//

#pragma once

#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <urdf_parser/urdf_parser.h>

#include <controller_interface/multi_interface_controller.h>
#include <cheetah_common/hardware_interface/hybrid_joint_interface.h>

#include <cheetah_msgs/LegsCmd.h>
#include <cheetah_msgs/LegsState.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include "cpp_types.h"
#include "state_estimate.h"

namespace cheetah_ros
{
class ControllerBase : public controller_interface::MultiInterfaceController<HybridJointInterface>
{
public:
  struct LegJoints
  {
    HybridJointHandle joints_[3];
  };
  struct LegCmd
  {
    ros::Time stamp_;
    Eigen::Vector3d foot_pos_des_, foot_vel_des_, ff_cartesian_;
    Eigen::Matrix3d kp_cartesian_, kd_cartesian_;
  };

  ControllerBase() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& /*time*/) override;
  virtual void updateData(const ros::Time& time, const ros::Duration& period);
  virtual void updateCommand(const ros::Time& time, const ros::Duration& period);

  LegJoints& getLegJoints(LegPrefix leg);
  void setLegCmd(LegPrefix leg, const LegCmd& cmd);

protected:
  void publishState(const ros::Time& time, const ros::Duration& period);

  RobotState robot_state_;
  std::shared_ptr<StateEstimateBase> state_estimate_;

  std::shared_ptr<urdf::ModelInterface> urdf_;
  std::shared_ptr<pinocchio::Model> pin_model_;
  std::shared_ptr<pinocchio::Data> pin_data_;

private:
  void legsCmdCallback(const cheetah_msgs::LegsCmd::ConstPtr& msg);

  LegJoints leg_joints_[4];
  LegCmd leg_cmd_[4];

  ros::Subscriber legs_cmd_sub_;
  realtime_tools::RealtimeBuffer<cheetah_msgs::LegsCmd> legs_cmd_buffer_;
  std::shared_ptr<realtime_tools::RealtimePublisher<cheetah_msgs::LegsState> > state_pub_;
  ros::Time last_publish_;
};

};  // namespace cheetah_ros
