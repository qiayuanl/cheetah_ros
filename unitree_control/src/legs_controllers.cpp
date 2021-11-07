//
// Created by qiayuan on 2021/11/6.
//
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <urdf_parser/urdf_parser.h>
#include <pluginlib/class_list_macros.hpp>

#include "unitree_control/legs_controllers.h"

namespace unitree_ros
{
LegsController::LegsController(std::shared_ptr<pinocchio::Model> model, std::shared_ptr<pinocchio::Data> data)
  : pin_model_(std::move(model)), pin_data_(std::move(data))
{
}

bool LegsController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  if (pin_model_ == nullptr && pin_data_ == nullptr)
  {
    // Get the URDF on param server, then build model and data
    std::string urdf_string;
    controller_nh.getParam("/robot_description", urdf_string);
    if (urdf_string.empty())
      return false;
    pin_model_ = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(urdf::parseURDF(urdf_string), pinocchio::JointModelFreeFlyer(), *pin_model_);
    pin_data_ = std::make_shared<pinocchio::Data>(*pin_model_);
  }
  // Setup joint handles. Ignore id 0 (universe joint) and id 1 (root joint).
  ROS_ASSERT(pin_model_->njoints == 14);
  HybridJointInterface* joint_interface = robot_hw->get<HybridJointInterface>();
  for (int leg = 0; leg < 4; ++leg)
    for (int j = 0; j < 3; ++j)
      datas_[leg].joints_[j] = joint_interface->getHandle(pin_model_->names[2 + leg * 3 + j]);
  return true;
}

void LegsController::update(const ros::Time& time, const ros::Duration& period)
{
  updateData();
}

void LegsController::updateData()
{
  Eigen::VectorXd q(pin_model_->nq), v(pin_model_->nv);
  for (int leg = 0; leg < 4; ++leg)
    for (int joint = 0; joint < 3; ++joint)
    {
      q(7 + leg * 3 + joint) = datas_[leg].joints_[joint].getPosition();
      v(6 + leg * 3 + joint) = datas_[leg].joints_[joint].getVelocity();
    }
  pinocchio::forwardKinematics(*pin_model_, *pin_data_, q, v);
  pinocchio::framesForwardKinematics(*pin_model_, *pin_data_, q);
  pinocchio::computeJointJacobians(*pin_model_, *pin_data_, q);

  if (last_publish_ + ros::Duration(0.5) < ros::Time::now())
  {
    last_publish_ = ros::Time::now();
    std::cout << std::setprecision(2) << pin_data_->oMf[pin_model_->getFrameId("FR_foot")].translation() << std::endl;
  }
}

}  // namespace unitree_ros

PLUGINLIB_EXPORT_CLASS(unitree_ros::LegsController, controller_interface::ControllerBase)
