//
// Created by qiayuan on 2021/11/17.
//

#pragma once

#include <ros/ros.h>
#include <cheetah_basic_controllers/cpp_types.h>
#include <cheetah_common/ros_utilities.h>

namespace cheetah_ros
{
template <typename T>
class OffsetDurationGait
{
public:
  using Ptr = std::shared_ptr<OffsetDurationGait>;
  OffsetDurationGait(T cycle, const Vec4<T>& offsets, const Vec4<T>& durations)
    : cycle_(cycle), offsets_(offsets), durations_(durations)
  {
  }

  void update(const ros::Time time)
  {
    phase_ = std::fmod(time.toSec() / cycle_, 1.);
  }

  DVec<T> getMpcTable(int horizon)
  {
    DVec<T> mpc_table(4 * horizon);
    int iteration = phase_ * horizon;

    for (int i = 0; i < horizon; i++)
    {
      int iter = (i + iteration) % horizon;
      for (int j = 0; j < 4; j++)
      {
        int progress = iter - offsets_[j] * horizon;
        if (progress < 0)
          progress += 1.;
        if (progress > durations_[j] * horizon)
          mpc_table[i * 4 + j] = 0;
        else
          mpc_table[i * 4 + j] = 1;
      }
    }
    return mpc_table;
  }

  Vec4<int> getContactState()
  {
    Vec4<int> state;

    for (int i = 0; i < 4; i++)
    {
      T progress = phase_ - offsets_[i];
      if (progress < 0)
        progress += 1.;
      if (progress > durations_[i])
        progress = 0;
      else
        progress = 1;
      state[i] = progress;
    }
    return state;
  }

  Vec4<T> getSwingTime()
  {
    Vec4<T> ones;
    ones.setOnes();
    return (ones - durations_) * cycle_;
  }

  Vec4<T> getStandTime()
  {
    return durations_ * cycle_;
  }

private:
  T cycle_, phase_;

  Vec4<T> offsets_;    // offset in 0.0 ~ 1.0
  Vec4<T> durations_;  // duration of step in 0.0 ~ 1.0
};

template <typename T>
class OffsetDurationGaitRos : public OffsetDurationGait<T>
{
public:
  OffsetDurationGaitRos<T>(XmlRpc::XmlRpcValue& params)
    : OffsetDurationGait<T>(xmlRpcGetDouble(params["cycle"]), getVec4(params, "offsets"), getVec4(params, "durations"))
  {
    ROS_ASSERT(params.hasMember("cycle"));
    ROS_ASSERT(params.hasMember("offsets"));
    ROS_ASSERT(params.hasMember("durations"));
  }

private:
  Vec4<T> getVec4(XmlRpc::XmlRpcValue& params, const std::string& name)
  {
    Vec4<T> array;
    if (params.hasMember(name))
      if (params[name].getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        if (params[name].size() == 4)
          array << xmlRpcGetDouble(params[name], 0), xmlRpcGetDouble(params[name], 1), xmlRpcGetDouble(params[name], 2),
              xmlRpcGetDouble(params[name], 3);
      }
    return array;
  }
};

}  // namespace cheetah_ros
