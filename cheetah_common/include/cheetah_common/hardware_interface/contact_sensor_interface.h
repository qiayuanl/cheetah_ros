//
// Created by qiayuan on 2021/11/5.
//
#pragma once
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>

namespace cheetah_ros
{
class ContactSensorHandle
{
public:
  ContactSensorHandle() = default;

  ContactSensorHandle(const std::string& name, const bool* is_contact) : is_contact_(is_contact)
  {
    if (!is_contact)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. is_contact pointer is null.");
    }
  }

  std::string getName() const
  {
    return name_;
  }

  bool getIsContact()
  {
    assert(is_contact_);
    return *is_contact_;
  }

private:
  std::string name_;

  const bool* is_contact_ = { nullptr };
};

class ContactSensorInterface
  : public hardware_interface::HardwareResourceManager<ContactSensorHandle, hardware_interface::DontClaimResources>
{
};

}  // namespace cheetah_ros
