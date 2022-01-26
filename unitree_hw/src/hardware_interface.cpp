
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
  setupJoints(root_nh);
  setupImu(root_nh);

  udp_ = std::make_shared<UNITREE_LEGGED_SDK::UDP>(UNITREE_LEGGED_SDK::LOWLEVEL);
  udp_->InitCmdData(low_cmd_);

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

  imu_data_.ori[0] = low_state.imu.quaternion[1];
  imu_data_.ori[1] = low_state.imu.quaternion[2];
  imu_data_.ori[2] = low_state.imu.quaternion[3];
  imu_data_.ori[3] = low_state.imu.quaternion[0];
  imu_data_.angular_vel[0] = low_state.imu.gyroscope[0];
  imu_data_.angular_vel[1] = low_state.imu.gyroscope[1];
  imu_data_.angular_vel[2] = low_state.imu.gyroscope[2];
  imu_data_.linear_acc[0] = low_state.imu.accelerometer[0];
  imu_data_.linear_acc[1] = low_state.imu.accelerometer[1];
  imu_data_.linear_acc[2] = low_state.imu.accelerometer[2];

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
  for (int i = 0; i < 20; ++i)
  {
    low_cmd_.motorCmd[i].q = joint_data_[i].pos_des_;
    low_cmd_.motorCmd[i].dq = joint_data_[i].vel_des_;
    low_cmd_.motorCmd[i].Kp = joint_data_[i].kp_;
    low_cmd_.motorCmd[i].Kd = joint_data_[i].kd_;
    low_cmd_.motorCmd[i].tau = joint_data_[i].ff_;
  }

  safety_->PositionLimit(low_cmd_);
  udp_->SetSend(low_cmd_);
  udp_->Send();
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
    if (joint.first.find("FR") != std::string::npos)
      leg_index = UNITREE_LEGGED_SDK::FR_;
    else if (joint.first.find("FL") != std::string::npos)
      leg_index = UNITREE_LEGGED_SDK::FL_;
    else if (joint.first.find("RR") != std::string::npos)
      leg_index = UNITREE_LEGGED_SDK::RR_;
    else if (joint.first.find("RL") != std::string::npos)
      leg_index = UNITREE_LEGGED_SDK::RL_;
    else
      continue;
    if (joint.first.find("hip") != std::string::npos)
      joint_index = 0;
    else if (joint.first.find("thigh") != std::string::npos)
      joint_index = 1;
    else if (joint.first.find("calf") != std::string::npos)
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
  registerInterface(&joint_state_interface_);
  registerInterface(&hybrid_joint_interface_);
  return true;
}

bool UnitreeHW::setupImu(ros::NodeHandle& root_nh)
{
  imu_sensor_interface_.registerHandle(hardware_interface::ImuSensorHandle(
      "unitree_imu", "unitree_imu", imu_data_.ori, imu_data_.ori_cov, imu_data_.angular_vel, imu_data_.angular_vel_cov,
      imu_data_.linear_acc, imu_data_.linear_acc_cov));
  registerInterface(&imu_sensor_interface_);
  return true;
}

}  // namespace cheetah_ros
