//
// Created by qiayuan on 2021/11/9.
//

#pragma once
#include "legs_controller.h"
#include "foot_swing_trajectory.h"
#include <unitree_msgs/FeetCmd.h>

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
    ros::Time take_off_time_;
    double phase_, swing_time_;
  };

  FeetController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void updateCommand(const ros::Time& time, const ros::Duration& period) override;
  State& getFootState(LegPrefix leg);
  void setSwing(LegPrefix leg, const Eigen::Vector3d& final_pos, double swing_time);
  void setTouch(LegPrefix leg, const Eigen::Vector3d& force);

private:
  void feetCmdCallback(const unitree_msgs::FeetCmd::ConstPtr& msg);

  FootSwingTrajectory<double> swing_trajectory_[4];
  State states_[4];

  ros::Subscriber feet_cmd_sub_;
  realtime_tools::RealtimeBuffer<unitree_msgs::FeetCmd> feet_cmd_buffer_;
};

}  // namespace unitree_ros
