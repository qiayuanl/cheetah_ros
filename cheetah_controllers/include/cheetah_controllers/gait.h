//
// Created by qiayuan on 2021/11/17.
//

#pragma once

#include <ros/ros.h>
#include <unitree_control/cpp_types.h>

namespace unitree_ros
{
template <typename T>
class OffsetDurationGait
{
public:
  OffsetDurationGait(int segment, T gait_cycle, const Vec4<T>& offsets, const Vec4<T>& durations)
    : segment_(segment), mpc_dt_(gait_cycle / static_cast<T>(segment_)), offsets_(offsets), durations_(durations)
  {
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
      Eigen::Array4i progress = iter - offsets_;
      for (int j = 0; j < 4; j++)
      {
        if (progress[j] < 0)
          progress[j] += segment_;
        if (progress[j] < durations_[j])
          mpc_table_[i * 4 + j] = 1;
        else
          mpc_table_[i * 4 + j] = 0;
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

private:
  int segment_, iteration_;
  T mpc_dt_;
  int* mpc_table_;

  Eigen::Array4d offsets_;    // offset in mpc segments
  Eigen::Array4d durations_;  // duration of step in mpc segments
};

}  // namespace unitree_ros
