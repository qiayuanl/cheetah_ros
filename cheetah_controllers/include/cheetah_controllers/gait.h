//
// Created by qiayuan on 2021/11/17.
//

#pragma once

#include <ros/ros.h>
#include <unitree_control/cpp_types.h>
#include <unitree_common/ros_utilities.h>

namespace unitree_ros
{
template <typename T>
class OffsetDurationGait
{
public:
  OffsetDurationGait(int segment, T cycle, const Vec4<T>& offsets, const Vec4<T>& durations)
    : segment_(segment), mpc_dt_(cycle / static_cast<T>(segment_)), offsets_(offsets), durations_(durations)
  {
    mpc_table_ = new int[segment_ * 4];
  }

  ~OffsetDurationGait()
  {
    delete[] mpc_table_;
  }

  void update(const ros::Time time)
  {
    iteration_ = std::fmod(time.toSec() / mpc_dt_, segment_);
  }

  int* getMpcTable()
  {
    for (int i = 0; i < segment_; i++)
    {
      int iter = (i + iteration_ + 1) % segment_;
      for (int j = 0; j < 4; j++)
      {
        int progress = iter - offsets_[i] * segment_;
        if (progress < 0)
          progress += segment_;
        if (progress > durations_[i] * segment_)
          mpc_table_[i * 4 + j] = 0;
        else
          mpc_table_[i * 4 + j] = 1;
      }
    }
    return mpc_table_;
  }

  Vec4<int> getContactState()
  {
    Vec4<int> state;

    for (int i = 0; i < 4; i++)
    {
      int progress = iteration_ - offsets_[i] * segment_;
      if (progress < 0)
        progress += segment_;
      if (progress > durations_[i] * segment_)
        progress = 0;
      else
        progress = 1;
      state[i] = progress;
    }
    return state;
  }

  Vec4<T> getSwingTime()
  {
    return durations_ * mpc_dt_ * segment_;
  }

  Vec4<T> getStandTime()
  {
    Vec4<T> ones;
    ones.setOnes();
    return (ones - durations_) * mpc_dt_ * segment_;
  }

  void resizeSegment(int segment)
  {
    delete[] mpc_table_;
    segment_ = segment;
    mpc_table_ = new int[segment_ * 4];
  }

private:
  int segment_, iteration_;
  T mpc_dt_;
  int* mpc_table_;

  Vec4<T> offsets_;    // offset in 0.0 ~ 1.0
  Vec4<T> durations_;  // duration of step in 0.0 ~ 1.0
};

template <typename T>
class OffsetDurationGaitRos : public OffsetDurationGait<T>
{
public:
  OffsetDurationGaitRos<T>(XmlRpc::XmlRpcValue& params, int segment)
  {
    ROS_ASSERT(params.hasMember("cycle"));
    ROS_ASSERT(params.hasMember("offsets"));
    ROS_ASSERT(params.hasMember("durations"));
    OffsetDurationGait<T>(segment, xmlRpcGetDouble(params["cycle"]), getVec4(params, "offsets"),
                          getVec4(params, "durations"));
  }

private:
  Vec4<T> getVec4(XmlRpc::XmlRpcValue& params, const std::string& name)
  {
    Vec4<T> array;
    if (params.hasMember(name))
      if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        if (params[name].size() == 4)
          array << xmlRpcGetDouble(params[name], 0), xmlRpcGetDouble(params[name], 1), xmlRpcGetDouble(params[name], 2),
              xmlRpcGetDouble(params[name], 3);
      }
    return array;
  }
};

}  // namespace unitree_ros
