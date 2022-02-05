
//
// Created by qiayuan on 1/24/22.
//

#pragma once

#include <vector>
#include <string>
#include <memory>

// ROS
#include <ros/ros.h>
#include <urdf/model.h>
#include <XmlRpcValue.h>

// ROS control
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <cheetah_common/hardware_interface/hybrid_joint_interface.h>
#include <cheetah_common/hardware_interface/contact_sensor_interface.h>
#include <cheetah_common/cpp_types.h>
#include <hardware_interface/imu_sensor_interface.h>

#include "unitree_legged_sdk/udp.h"
#include "unitree_legged_sdk/safety.h"

namespace cheetah_ros
{
struct UnitreeMotorData
{
  double pos_, vel_, tau_;                   // state
  double pos_des_, vel_des_, kp_, kd_, ff_;  // command
};

struct UnitreeImuData
{
  double ori[4];
  double ori_cov[9];
  double angular_vel[3];
  double angular_vel_cov[9];
  double linear_acc[3];
  double linear_acc_cov[9];
};

class UnitreeHW : public hardware_interface::RobotHW
{
public:
  UnitreeHW() = default;
  /** \brief Get necessary params from param server. Init hardware_interface.
   *
   * Get params from param server and check whether these params are set. Load urdf of robot. Set up transmission and
   * joint limit. Get configuration of can bus and create data pointer which point to data received from Can bus.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True when init successful, False when failed.
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  /** \brief Communicate with hardware. Get data, status of robot.
   *
   * Call @ref UNITREE_LEGGED_SDK::UDP::Recv() to get robot's state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void read(const ros::Time& time, const ros::Duration& period) override;

  /** \brief Comunicate with hardware. Publish command to robot.
   *
   * Propagate joint state to actuator state for the stored
   * transmission. Limit cmd_effort into suitable value. Call @ref UNITREE_LEGGED_SDK::UDP::Recv(). Publish actuator
   * current state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void write(const ros::Time& time, const ros::Duration& period) override;

private:
  /** \brief Load urdf of robot from param server.
   *
   * Load urdf of robot from param server.
   *
   * @param root_nh Root node-handle of a ROS node
   * @return True if successful.
   */
  bool loadUrdf(ros::NodeHandle& root_nh);

  /** \brief Set up Joints.
   *
   * Set up transmission
   *
   * @return True if successful.
   */
  bool setupJoints();

  bool setupImu();

  bool setupContactSensor(ros::NodeHandle& nh);

  std::shared_ptr<UNITREE_LEGGED_SDK::UDP> udp_;
  std::shared_ptr<UNITREE_LEGGED_SDK::Safety> safety_;
  UNITREE_LEGGED_SDK::LowState low_state_{};
  UNITREE_LEGGED_SDK::LowCmd low_cmd_{};

  UnitreeMotorData joint_data_[20]{};
  UnitreeImuData imu_data_{};
  bool contact_state_[4]{};
  int contact_threshold_{};

  // Interface
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::ImuSensorInterface imu_sensor_interface_;
  HybridJointInterface hybrid_joint_interface_;
  ContactSensorInterface contact_sensor_interface_;

  // URDF model of the robot
  std::string urdf_string_;                  // for transmission
  std::shared_ptr<urdf::Model> urdf_model_;  // for limit

  //  ros::Time last_publish_time_;
  //  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::ActuatorState>> actuator_state_pub_;
};

}  // namespace cheetah_ros
