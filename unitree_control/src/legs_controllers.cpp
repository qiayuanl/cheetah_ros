//
// Created by qiayuan on 2021/11/6.
//
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <urdf_parser/urdf_parser.h>
#include <pluginlib/class_list_macros.hpp>

#include "unitree_control/legs_controllers.h"

namespace unitree_ros
{
LegsController::LegsController(std::shared_ptr<pinocchio::Model> model, std::shared_ptr<pinocchio::Data> data)
  : model_(std::move(model)), data_(std::move(data))
{
}

bool LegsController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  if (model_ == nullptr && data_ == nullptr)
  {
    // Get the URDF on param server, then build model and data
    std::string urdf_string;
    controller_nh.getParam("/robot_description", urdf_string);
    if (urdf_string.empty())
      return false;
    model_ = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(urdf::parseURDF(urdf_string), *model_);
    data_ = std::make_shared<pinocchio::Data>(*model_);
  }

  HybridJointInterface* joint_interface = robot_hw->get<HybridJointInterface>();
  for (int id = 0; id < model_->njoints; ++id)
  {
    std::string name = model_->names[id];
    if (name != "universe")
      joint_handles_.push_back(joint_interface->getHandle(name));
  }
  return true;
}

void LegsController::update(const ros::Time& time, const ros::Duration& period)
{
}

}  // namespace unitree_ros

PLUGINLIB_EXPORT_CLASS(unitree_ros::LegsController, controller_interface::ControllerBase)
