
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
  setupJoints();
  setupImu();
  setupContactSensor(robot_hw_nh);

  udp_ = std::make_shared<UNITREE_LEGGED_SDK::UDP>(UNITREE_LEGGED_SDK::LOWLEVEL);
  udp_->InitCmdData(low_cmd_);

  safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::Aliengo);

  actuator_state_pub_.reset(
      new realtime_tools::RealtimePublisher<cheetah_msgs::MotorState>(root_nh, "/motor_states", 100));
  return true;
}

void UnitreeHW::read(const ros::Time& time, const ros::Duration& period)
{
  udp_->Recv();
  udp_->GetRecv(low_state_);

  for (int i = 0; i < 20; ++i)
  {
    joint_data_[i].pos_ = low_state_.motorState[i].q;
    joint_data_[i].vel_ = low_state_.motorState[i].dq;
    joint_data_[i].tau_ = low_state_.motorState[i].tauEst;
  }

  imu_data_.ori[0] = low_state_.imu.quaternion[1];
  imu_data_.ori[1] = low_state_.imu.quaternion[2];
  imu_data_.ori[2] = low_state_.imu.quaternion[3];
  imu_data_.ori[3] = low_state_.imu.quaternion[0];
  imu_data_.angular_vel[0] = low_state_.imu.gyroscope[0];
  imu_data_.angular_vel[1] = low_state_.imu.gyroscope[1];
  imu_data_.angular_vel[2] = low_state_.imu.gyroscope[2];
  imu_data_.linear_acc[0] = low_state_.imu.accelerometer[0];
  imu_data_.linear_acc[1] = low_state_.imu.accelerometer[1];
  imu_data_.linear_acc[2] = low_state_.imu.accelerometer[2];

  contact_state_[LegPrefix::FL] = low_state_.footForce[UNITREE_LEGGED_SDK::FL_] > contact_threshold_;
  contact_state_[LegPrefix::FR] = low_state_.footForce[UNITREE_LEGGED_SDK::FR_] > contact_threshold_;
  contact_state_[LegPrefix::RL] = low_state_.footForce[UNITREE_LEGGED_SDK::RL_] > contact_threshold_;
  contact_state_[LegPrefix::RR] = low_state_.footForce[UNITREE_LEGGED_SDK::RR_] > contact_threshold_;

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
  publishMotorState(time);
}

bool UnitreeHW::loadUrdf(ros::NodeHandle& root_nh)
{
  if (urdf_model_ == nullptr)
    urdf_model_ = std::make_shared<urdf::Model>();
  // get the urdf param on param server
  root_nh.getParam("/robot_description", urdf_string_);
  return !urdf_string_.empty() && urdf_model_->initString(urdf_string_);
}

bool UnitreeHW::setupJoints()
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

bool UnitreeHW::setupImu()
{
  imu_sensor_interface_.registerHandle(hardware_interface::ImuSensorHandle(
      "unitree_imu", "unitree_imu", imu_data_.ori, imu_data_.ori_cov, imu_data_.angular_vel, imu_data_.angular_vel_cov,
      imu_data_.linear_acc, imu_data_.linear_acc_cov));
  registerInterface(&imu_sensor_interface_);
  return true;
}

bool UnitreeHW::setupContactSensor(ros::NodeHandle& nh)
{
  nh.getParam("contact_threshold", contact_threshold_);
  contact_sensor_interface_.registerHandle(ContactSensorHandle("feet", contact_state_));
  registerInterface(&contact_sensor_interface_);
  return true;
}

void UnitreeHW::publishMotorState(const ros::Time& time)
{
  if (last_publish_time_ + ros::Duration(1.0 / 100.0) < time)
  {
    if (actuator_state_pub_->trylock())
    {
      cheetah_msgs::MotorState motor_state;
      motor_state.header.stamp = time;
      for (int i = 0; i < 20; ++i)
      {
        motor_state.q[i] = low_state_.motorState[i].q;
        motor_state.dq[i] = low_state_.motorState[i].dq;
        motor_state.tau[i] = low_state_.motorState[i].tauEst;
        motor_state.temperature[i] = low_state_.motorState[i].temperature;
        motor_state.q_des[i] = low_cmd_.motorCmd[i].q;
        motor_state.dq_des[i] = low_cmd_.motorCmd[i].dq;
        motor_state.ff[i] = low_cmd_.motorCmd[i].tau;
      }
      actuator_state_pub_->msg_ = motor_state;
      actuator_state_pub_->unlockAndPublish();
      last_publish_time_ = time;
    }
  }
}

}  // namespace cheetah_ros
