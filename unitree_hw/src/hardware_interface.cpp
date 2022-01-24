
//
// Created by qiayuan on 1/24/22.
//

#include "unitree_hw/hardware_interface.h"

namespace cheetah_ros
{
bool UnitreeHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  if (!loadUrdf(root_nh))
  {
    ROS_ERROR("Error occurred while setting up urdf");
    return false;
  }

  UNITREE_LEGGED_SDK::LowCmd cmd{};
  udp_ = std::make_shared<UNITREE_LEGGED_SDK::UDP>(UNITREE_LEGGED_SDK::LOWLEVEL);
  udp_->InitCmdData(cmd);
  safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::Aliengo);

  // TODO Unitree motor publish
  return true;
}

void UnitreeHW::read(const ros::Time& time, const ros::Duration& period)
{
  udp_->Recv();
  UNITREE_LEGGED_SDK::LowState low_state;
  udp_->GetRecv(low_state);

  for (int i = 0; i < 20; ++i)
  {
    joint_data_[i].pos_ = low_state.motorState[i].q;
    joint_data_[i].vel_ = low_state.motorState[i].dq;
    joint_data_[i].tau_ = low_state.motorState[i].tauEst;
  }
  // Set feedforward and velocity cmd to zero to avoid for saft when not controller setCommand
  std::vector<std::string> names = hybrid_joint_interface_.getNames();
  for (const auto& name : names)
  {
    HybridJointHandle handle = hybrid_joint_interface_.getHandle(name);
    handle.setFeedforward(0.);
    handle.setVelocityDesired(0.);
  }
}

void UnitreeHW::write(const ros::Time& time, const ros::Duration& period)
{
  UNITREE_LEGGED_SDK::LowCmd cmd{};
  for (int i = 0; i < 20; ++i)
  {
    cmd.motorCmd[i].q = joint_data_[i].pos_des_;
    cmd.motorCmd[i].dq = joint_data_[i].vel_des_;
    cmd.motorCmd[i].Kp = joint_data_[i].kp_;
    cmd.motorCmd[i].Kd = joint_data_[i].kd_;
    cmd.motorCmd[i].tau = joint_data_[i].tau_;
  }

  safety_->PositionLimit(cmd);
  //  udp_->SetSend(cmd);
  //  udp_->Send();
}

bool UnitreeHW::loadUrdf(ros::NodeHandle& root_nh)
{
  if (urdf_model_ == nullptr)
    urdf_model_ = std::make_shared<urdf::Model>();
  // get the urdf param on param server
  root_nh.getParam("/robot_description", urdf_string_);
  return !urdf_string_.empty() && urdf_model_->initString(urdf_string_);
}

bool UnitreeHW::setupJoints(ros::NodeHandle& root_nh)
{
  for (const auto& joint : urdf_model_->joints_)
  {
    int leg_index, joint_index;
    if (joint.first.find("FR"))
      leg_index = UNITREE_LEGGED_SDK::FR_;
    else if (joint.first.find("FL"))
      leg_index = UNITREE_LEGGED_SDK::FL_;
    else if (joint.first.find("RR"))
      leg_index = UNITREE_LEGGED_SDK::RR_;
    else if (joint.first.find("RL"))
      leg_index = UNITREE_LEGGED_SDK::RL_;
    else
      continue;
    if (joint.first.find("hip"))
      joint_index = 0;
    else if (joint.first.find("thigh"))
      joint_index = 1;
    else if (joint.first.find("calf"))
      joint_index = 2;
    else
      continue;

    int index = leg_index * 3 + joint_index;
    hardware_interface::JointStateHandle state_handle(joint.first, &joint_data_[index].pos_, &joint_data_[index].vel_,
                                                      &joint_data_[index].tau_);
    joint_state_interface_.registerHandle(state_handle);
    hybrid_joint_interface_.registerHandle(HybridJointHandle(state_handle, &joint_data_[index].pos_des_,
                                                             &joint_data_[index].vel_des_, &joint_data_[index].kp_,
                                                             &joint_data_[index].kd_, &joint_data_[index].ff_));
  }
  return true;
}

}  // namespace cheetah_ros
