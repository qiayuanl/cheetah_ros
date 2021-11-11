//
// Created by qiayuan on 2021/11/9.
//
#include "unitree_control/feet_controller.h"

#include <unitree_common/ros_utilities.h>
#include <pluginlib/class_list_macros.hpp>

namespace unitree_ros
{
bool FeetController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  if (!LegsController::init(robot_hw, controller_nh))
    return false;
  K k;
  XmlRpc::XmlRpcValue feet_params;
  controller_nh.getParam("feet", feet_params);
  ROS_ASSERT(feet_params.getType() == XmlRpc::XmlRpcValue::TypeStruct)

  k.kp_stand_ = initK(feet_params, "kp_stand");
  k.kd_stand_ = initK(feet_params, "kd_stand");
  k.kp_swing_ = initK(feet_params, "kp_swing");
  k.kd_swing_ = initK(feet_params, "kd_swing");
  k_buffer.initRT(k);

  // ROS Topic
  feet_cmd_sub_ =
      controller_nh.subscribe<unitree_msgs::FeetCmd>("/cmd_feet", 1, &FeetController::feetCmdCallback, this);

  // Dynamic reconfigure
  dynamic_srv_ = std::make_shared<dynamic_reconfigure::Server<unitree_ros::FeetConfig>>(controller_nh);
  dynamic_reconfigure::Server<unitree_ros::FeetConfig>::CallbackType cb = [this](auto&& PH1, auto&& PH2) {
    dynamicCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
  };
  dynamic_initialized_ = false;
  dynamic_srv_->setCallback(cb);
  return true;
}

void FeetController::updateCommand(const ros::Time& time, const ros::Duration& period)
{
  // Update Command from ROS topic interface.
  unitree_msgs::FeetCmd cmd_msgs = *feet_cmd_buffer_.readFromRT();
  for (size_t j = 0; j < cmd_msgs.leg_prefix.size(); ++j)
  {
    State& state = states_[cmd_msgs.leg_prefix[j].prefix];
    if (cmd_msgs.header.stamp > state.cmd_time_)
    {
      state.cmd_time_ = cmd_msgs.header.stamp;
      if (cmd_msgs.touch_state[j] == cmd_msgs.SWING)
      {
        Eigen::Vector3d pos_final(cmd_msgs.pos_final[j].x, cmd_msgs.pos_final[j].y, cmd_msgs.pos_final[j].z);
        setSwing(LegPrefix(cmd_msgs.leg_prefix[j].prefix), pos_final, cmd_msgs.height[j], cmd_msgs.swing_time[j]);
      }
      else
      {
        Eigen::Vector3d force(cmd_msgs.ground_reaction_force[j].x, cmd_msgs.ground_reaction_force[j].y,
                              cmd_msgs.ground_reaction_force[j].z);
        setStand(LegPrefix(cmd_msgs.leg_prefix[j].prefix), force);
      }
    }
  }

  K k = *k_buffer.readFromRT();
  for (int leg = 0; leg < 4; ++leg)
  {
    if (states_[leg].touch_state_ == SWING)
    {
      LegsController::Command leg_cmd;
      states_[leg].phase_ += period.toSec() / states_[leg].swing_time_;
      if (states_[leg].phase_ > 1.)
        states_[leg].phase_ = 1.;
      swing_trajectory_[leg].computeSwingTrajectoryBezier(states_[leg].phase_, states_[leg].swing_time_);
      leg_cmd.stamp_ = time;
      leg_cmd.foot_pos_des_ = swing_trajectory_[leg].getPosition();
      leg_cmd.foot_vel_des_ = swing_trajectory_[leg].getVelocity();
      leg_cmd.kp_cartesian_ = k.kp_swing_;
      leg_cmd.kd_cartesian_ = k.kd_swing_;
      setLegCmd(LegPrefix(leg), leg_cmd);
    }
  }
  LegsController::updateCommand(time, period);
}

FeetController::State& FeetController::getFootState(LegPrefix leg)
{
  return states_[leg];
}

void FeetController::setSwing(LegPrefix leg, const Eigen::Vector3d& final_pos, double height, double swing_time)
{
  states_[leg].touch_state_ = SWING;
  states_[leg].phase_ = 0.;
  states_[leg].swing_time_ = swing_time;
  swing_trajectory_[leg].setHeight(height);
  swing_trajectory_[leg].setInitialPosition(getLegState(leg).foot_pos_);
  swing_trajectory_[leg].setFinalPosition(final_pos);
}

void FeetController::setStand(LegPrefix leg, const Eigen::Vector3d& force)
{
  states_[leg].touch_state_ = STAND;
  K k = *k_buffer.readFromRT();
  LegsController::Command leg_cmd;
  leg_cmd.foot_pos_des_ = getLegState(leg).foot_pos_;
  leg_cmd.foot_vel_des_.setZero();
  leg_cmd.kp_cartesian_ = k.kp_stand_;
  leg_cmd.kd_cartesian_ = k.kd_stand_;
  leg_cmd.ff_cartesian_ = -force;  // The ground reaction force is negative to the force of end-effector (foot)
  setLegCmd(leg, leg_cmd);
}

Eigen::Matrix3d FeetController::initK(XmlRpc::XmlRpcValue& feet_params, const std::string& name)
{
  Eigen::Matrix3d k;
  k.setZero();

  if (feet_params.hasMember(name))
    if (feet_params[name].getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      if (feet_params[name].size() == 3)
        k << xmlRpcGetDouble(feet_params[name], 0), 0., 0., 0., xmlRpcGetDouble(feet_params[name], 1), 0., 0., 0.,
            xmlRpcGetDouble(feet_params[name], 2);
    }

  return k;
}

void FeetController::feetCmdCallback(const unitree_msgs::FeetCmd::ConstPtr& msg)
{
  feet_cmd_buffer_.writeFromNonRT(*msg);
}

void FeetController::dynamicCallback(FeetConfig& config, uint32_t /*unused*/)
{
  if (!dynamic_initialized_)
  {
    ROS_INFO("[Feet] Dynamic params are set by rosparams at initialization");

    K init_k = *k_buffer.readFromNonRT();
    config.kp_stand_x = init_k.kp_stand_(0, 0);
    config.kp_stand_y = init_k.kp_stand_(1, 1);
    config.kp_stand_z = init_k.kp_stand_(2, 2);
    config.kd_stand_x = init_k.kd_stand_(0, 0);
    config.kd_stand_y = init_k.kd_stand_(1, 1);
    config.kd_stand_z = init_k.kd_stand_(2, 2);
    config.kp_swing_x = init_k.kp_swing_(0, 0);
    config.kp_swing_y = init_k.kp_swing_(1, 1);
    config.kp_swing_z = init_k.kp_swing_(2, 2);
    config.kd_swing_x = init_k.kd_swing_(0, 0);
    config.kd_swing_y = init_k.kd_swing_(1, 1);
    config.kd_swing_z = init_k.kd_swing_(2, 2);
    dynamic_initialized_ = true;
    return;
  }
  ROS_INFO("[Feet] Dynamic params update");

  K k;
  k.kp_stand_(0, 0) = config.kp_stand_x;
  k.kp_stand_(1, 1) = config.kp_stand_y;
  k.kp_stand_(2, 2) = config.kp_stand_z;
  k.kd_stand_(0, 0) = config.kd_stand_x;
  k.kd_stand_(1, 1) = config.kd_stand_y;
  k.kd_stand_(2, 2) = config.kd_stand_z;
  k.kp_swing_(0, 0) = config.kp_swing_x;
  k.kp_swing_(1, 1) = config.kp_swing_x;
  k.kp_swing_(2, 2) = config.kp_swing_x;
  k.kd_swing_(0, 0) = config.kd_swing_x;
  k.kd_swing_(1, 1) = config.kd_swing_x;
  k.kd_swing_(2, 2) = config.kd_swing_x;

  k_buffer.writeFromNonRT(k);
}

}  // namespace unitree_ros

PLUGINLIB_EXPORT_CLASS(unitree_ros::FeetController, controller_interface::ControllerBase)
