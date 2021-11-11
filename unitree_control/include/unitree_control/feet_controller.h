//
// Created by qiayuan on 2021/11/9.
//

#pragma once
#include "legs_controller.h"
#include "foot_swing_trajectory.h"
#include <unitree_msgs/FeetCmd.h>
#include <unitree_control/FeetConfig.h>
#include <dynamic_reconfigure/server.h>

namespace unitree_ros
{
class FeetController : public LegsController
{
public:
  enum TouchState
  {
    STAND = 0,
    SWING
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
  State& getFootState(LegPrefix leg);
  void setSwing(LegPrefix leg, const Eigen::Vector3d& final_pos, double height, double swing_time);
  void setStand(LegPrefix leg, const Eigen::Vector3d& force);

private:
  Eigen::Matrix3d initK(XmlRpc::XmlRpcValue& feet_params, const std::string& name);
  void feetCmdCallback(const unitree_msgs::FeetCmd::ConstPtr& msg);
  void dynamicCallback(unitree_ros::FeetConfig& config, uint32_t /*level*/);

  FootSwingTrajectory<double> swing_trajectory_[4];
  State states_[4];

  // ROS Topic interface
  ros::Subscriber feet_cmd_sub_;
  realtime_tools::RealtimeBuffer<unitree_msgs::FeetCmd> feet_cmd_buffer_;
  // Dynamic reconfigure
  realtime_tools::RealtimeBuffer<K> k_buffer;
  std::shared_ptr<dynamic_reconfigure::Server<unitree_ros::FeetConfig>> dynamic_srv_{};
  bool dynamic_initialized_;
};

}  // namespace unitree_ros
