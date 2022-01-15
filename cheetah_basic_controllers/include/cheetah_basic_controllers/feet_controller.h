//
// Created by qiayuan on 2021/11/9.
//

#pragma once
#include "controller_base.h"
#include "foot_swing_trajectory.h"
#include <cheetah_msgs/FeetCmd.h>
#include <cheetah_basic_controllers/FeetConfig.h>
#include <dynamic_reconfigure/server.h>

namespace cheetah_ros
{
class FeetController : public ControllerBase
{
public:
  enum TouchState
  {
    SWING = 0,
    STAND,
  };
  struct State
  {
    TouchState touch_state_;
    ros::Time cmd_time_;
    double phase_, swing_time_;
  };
  struct K
  {
    Eigen::Matrix3d kp_swing_, kd_swing_, kp_stand_, kd_stand_;
  };

  FeetController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void updateCommand(const ros::Time& time, const ros::Duration& period) override;
  TouchState getFootState(LegPrefix leg);
  void setSwing(LegPrefix leg, const Eigen::Vector3d& final_pos, double height, double swing_time);
  void setStand(LegPrefix leg, const Eigen::Vector3d& force);

private:
  Eigen::Matrix3d initK(XmlRpc::XmlRpcValue& feet_params, const std::string& name);
  void feetCmdCallback(const cheetah_msgs::FeetCmd::ConstPtr& msg);
  void dynamicCallback(cheetah_ros::FeetConfig& config, uint32_t /*level*/);

  FootSwingTrajectory<double> swing_trajectory_[4];
  State states_[4];

  // ROS Topic interface
  ros::Subscriber feet_cmd_sub_;
  realtime_tools::RealtimeBuffer<cheetah_msgs::FeetCmd> feet_cmd_buffer_;
  // Dynamic reconfigure
  realtime_tools::RealtimeBuffer<K> k_buffer;
  std::shared_ptr<dynamic_reconfigure::Server<cheetah_ros::FeetConfig>> dynamic_srv_{};
  bool dynamic_initialized_;
};

}  // namespace cheetah_ros
