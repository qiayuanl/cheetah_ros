//
// Created by qiayuan on 2021/11/15.
//

#pragma once
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <tf2_ros/transform_broadcaster.h>

#include "cpp_types.h"

namespace cheetah_ros
{
class StateEstimateBase
{
public:
  StateEstimateBase(ros::NodeHandle& nh);
  virtual ~StateEstimateBase(){};
  virtual void update(ros::Time time, RobotState& state);

private:
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;
  ros::Time last_publish_;
  tf2_ros::TransformBroadcaster tf_br_;
};

class FromTopicStateEstimate : public StateEstimateBase
{
public:
  FromTopicStateEstimate(ros::NodeHandle& nh);
  ~FromTopicStateEstimate() override{};

  void update(ros::Time time, RobotState& state) override;

private:
  void callback(const nav_msgs::Odometry::ConstPtr& msg);

  ros::Subscriber sub_;
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_;
};

class LinearKFPosVelEstimator : public StateEstimateBase
{
public:
  LinearKFPosVelEstimator(ros::NodeHandle& nh);
  void update(ros::Time time, RobotState& state) override;

private:
  Eigen::Matrix<double, 18, 1> x_hat_;
  Eigen::Matrix<double, 12, 1> ps_;
  Eigen::Matrix<double, 12, 1> vs_;
  Eigen::Matrix<double, 18, 18> a_;
  Eigen::Matrix<double, 18, 18> q_;
  Eigen::Matrix<double, 18, 18> p_;
  Eigen::Matrix<double, 28, 28> r_;
  Eigen::Matrix<double, 18, 3> b_;
  Eigen::Matrix<double, 28, 18> c_;
};

class ImuSensorEstimator : public StateEstimateBase
{
public:
  using StateEstimateBase::StateEstimateBase;
  ImuSensorEstimator(ros::NodeHandle& nh, hardware_interface::ImuSensorHandle imu);
  void update(ros::Time time, RobotState& state) override;

private:
  hardware_interface::ImuSensorHandle imu_;
};

template <typename T>
Mat3<T> quaternionToRotationMatrix(const Eigen::Quaternion<T>& q)
{
  T w = q.w();
  T x = q.x();
  T y = q.y();
  T z = q.z();
  Mat3<T> r;
  r << 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y), 2 * (x * y + w * z), 1 - 2 * (x * x + z * z),
      2 * (y * z - w * x), 2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y);
  return r;
}

}  // namespace cheetah_ros
