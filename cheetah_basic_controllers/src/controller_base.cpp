//
// Created by qiayuan on 2021/11/6.
//

#include "cheetah_basic_controllers/controller_base.h"

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace cheetah_ros
{
bool ControllerBase::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  if (pin_model_ == nullptr && pin_data_ == nullptr)
  {
    // Get the URDF on param server, then build model and data
    std::string urdf_string;
    controller_nh.getParam("/robot_description", urdf_string);
    if (urdf_string.empty())
      return false;
    pin_model_ = std::make_shared<pinocchio::Model>();
    urdf_ = urdf::parseURDF(urdf_string);
    pinocchio::urdf::buildModel(urdf_, pinocchio::JointModelFreeFlyer(), *pin_model_);
    pin_data_ = std::make_shared<pinocchio::Data>(*pin_model_);
  }
  // Setup joint handles. Ignore id 0 (universe joint) and id 1 (root joint).
  HybridJointInterface* hybrid_joint_interface = robot_hw->get<HybridJointInterface>();
  for (int leg = 0; leg < 4; ++leg)
    for (int j = 0; j < 3; ++j)
      leg_joints_[leg].joints_[j] = hybrid_joint_interface->getHandle(pin_model_->names[2 + leg * 3 + j]);

  // ROS Topic
  legs_cmd_sub_ =
      controller_nh.subscribe<cheetah_msgs::LegsCmd>("/cmd_legs", 1, &ControllerBase::legsCmdCallback, this);
  state_pub_ =
      std::make_shared<realtime_tools::RealtimePublisher<cheetah_msgs::LegsState>>(controller_nh, "/leg_states", 100);

  state_estimate_ = std::make_shared<GroundTruth>(controller_nh);  // TODO add interface
  return true;
}

void ControllerBase::update(const ros::Time& time, const ros::Duration& period)
{
  updateData(time, period);
  updateCommand(time, period);
  publishState(time, period);
}

void ControllerBase::updateData(const ros::Time& time, const ros::Duration& period)
{
  if (state_estimate_ != nullptr)
    state_estimate_->update(robot_state_);

  Eigen::VectorXd q(pin_model_->nq), v(pin_model_->nv);
  for (int leg = 0; leg < 4; ++leg)
    for (int joint = 0; joint < 3; ++joint)
    {
      // Free-flyer joints have 6 degrees of freedom, but are represented by 7 scalars: the position of the basis center
      // in the world frame, and the orientation of the basis in the world frame stored as a quaternion.
      q(7 + leg * 3 + joint) = leg_joints_[leg].joints_[joint].getPosition();
      v(6 + leg * 3 + joint) = leg_joints_[leg].joints_[joint].getVelocity();
    }
  q.head(7) << robot_state_.pos_, robot_state_.quat_.coeffs();
  v.head(6) << robot_state_.linear_vel_, robot_state_.angular_vel_;

  pinocchio::forwardKinematics(*pin_model_, *pin_data_, q, v);
  pinocchio::computeJointJacobians(*pin_model_, *pin_data_);
  pinocchio::updateFramePlacements(*pin_model_, *pin_data_);
  for (int leg = 0; leg < 4; ++leg)
  {
    pinocchio::FrameIndex frame_id = pin_model_->getFrameId(LEG_PREFIX[leg] + "_foot");
    robot_state_.foot_pos_[leg] = pin_data_->oMf[frame_id].translation();
    robot_state_.foot_vel_[leg] =
        pinocchio::getFrameVelocity(*pin_model_, *pin_data_, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED)
            .linear();
  }
}

void ControllerBase::updateCommand(const ros::Time& time, const ros::Duration& period)
{
  // Update Command from ROS topic interface.
  cheetah_msgs::LegsCmd cmd_msgs = *legs_cmd_buffer_.readFromRT();
  for (size_t j = 0; j < cmd_msgs.leg_prefix.size(); ++j)
  {
    LegCmd& cmd = leg_cmd_[cmd_msgs.leg_prefix[j].prefix];
    if (cmd_msgs.header.stamp >= cmd.stamp_)
    {
      cmd.stamp_ = cmd_msgs.header.stamp;
      cmd.foot_pos_des_ << cmd_msgs.foot_pos_des[j].x, cmd_msgs.foot_pos_des[j].y, cmd_msgs.foot_pos_des[j].z;
      cmd.foot_vel_des_ << cmd_msgs.foot_vel_des[j].x, cmd_msgs.foot_vel_des[j].y, cmd_msgs.foot_vel_des[j].z;
      cmd.kp_cartesian_ << cmd_msgs.kp_cartesian[j].x, 0., 0., 0., cmd_msgs.kp_cartesian[j].y, 0., 0., 0.,
          cmd_msgs.kp_cartesian[j].z;
      cmd.kd_cartesian_ << cmd_msgs.kd_cartesian[j].x, 0., 0., 0., cmd_msgs.kd_cartesian[j].y, 0., 0., 0.,
          cmd_msgs.kd_cartesian[j].z;
    }
  }

  // Update joint space command
  for (int leg = 0; leg < 4; ++leg)
  {
    Eigen::Vector3d foot_force = leg_cmd_[leg].ff_cartesian_;
    // cartesian PD
    foot_force += leg_cmd_[leg].kp_cartesian_ * (leg_cmd_[leg].foot_pos_des_ - robot_state_.foot_pos_[leg]);
    foot_force += leg_cmd_[leg].kd_cartesian_ * (leg_cmd_[leg].foot_vel_des_ - robot_state_.foot_vel_[leg]);
    Eigen::Matrix<double, 6, 18> jac;
    pinocchio::getFrameJacobian(*pin_model_, *pin_data_, pin_model_->getFrameId(LEG_PREFIX[leg] + "_foot"),
                                pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jac);
    Eigen::Matrix<double, 6, 1> wrench;
    wrench.setZero();
    wrench.head(3) = foot_force;
    Eigen::Matrix<double, 18, 1> tau = jac.transpose() * wrench;
    for (int joint = 0; joint < 3; ++joint)
      leg_joints_[leg].joints_[joint].setFeedforward(tau(6 + leg * 3 + joint));
  }
}

ControllerBase::LegJoints& ControllerBase::getLegJoints(LegPrefix leg)
{
  return leg_joints_[leg];
}

void ControllerBase::setLegCmd(LegPrefix leg, const LegCmd& cmd)
{
  leg_cmd_[leg] = cmd;
}

void ControllerBase::publishState(const ros::Time& time, const ros::Duration& period)
{
  if (time < ros::Time(0.01))  // When simulate time reset
    last_publish_ = time;
  if (time - last_publish_ > ros::Duration(0.01))  // 100Hz
  {
    last_publish_ = time;
    cheetah_msgs::LegsState legs_state;
    legs_state.header.stamp = time;
    if (state_pub_->trylock())
    {
      for (int leg = 0; leg < 4; ++leg)
      {
        state_pub_->msg_.foot_pos[leg].x = robot_state_.foot_pos_[leg].x();
        state_pub_->msg_.foot_pos[leg].y = robot_state_.foot_pos_[leg].y();
        state_pub_->msg_.foot_pos[leg].z = robot_state_.foot_pos_[leg].z();
        state_pub_->msg_.foot_vel[leg].x = robot_state_.foot_vel_[leg].x();
        state_pub_->msg_.foot_vel[leg].y = robot_state_.foot_vel_[leg].y();
        state_pub_->msg_.foot_vel[leg].z = robot_state_.foot_vel_[leg].z();
      }
      state_pub_->unlockAndPublish();
    }
  }
}

void ControllerBase::legsCmdCallback(const cheetah_msgs::LegsCmd::ConstPtr& msg)
{
  legs_cmd_buffer_.writeFromNonRT(*msg);
}

}  // namespace cheetah_ros

PLUGINLIB_EXPORT_CLASS(cheetah_ros::ControllerBase, controller_interface::ControllerBase)
