//
// Created by qiayuan on 2021/11/17.
//

#pragma once

#include <ros/ros.h>
#include <unitree_control/cpp_types.h>

namespace unitree_ros
{
class OffsetDurationGait
{
public:
  OffsetDurationGait(int segment, double gait_cycle, const Vec4<int>& offsets, const Vec4<int>& durations)
    : segment_(segment)
    , gait_cycle_(gait_cycle)
    , mpc_dt_(gait_cycle / static_cast<double>(segment_))
    , offsets_(offsets)
    , durations_(durations)
    , offsets_phase_(offsets.cast<double>() / (double)segment_)
    , durations_phase_(durations.cast<double>() / (double)segment_)
  {
  }
  ~OffsetDurationGait()
  {
    delete[] mpc_table_;
  }
  void update(const ros::Time time)
  {
    iteration_ = static_cast<int>(std::fmod(time.toSec() / mpc_dt_, segment_));
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

private:
  int segment_, iteration_;
  double gait_cycle_, mpc_dt_;
  int* mpc_table_;

  Eigen::Array4i offsets_;          // offset in mpc segments
  Eigen::Array4i durations_;        // duration of step in mpc segments
  Eigen::Array4d offsets_phase_;    // offsets in phase (0 to 1)
  Eigen::Array4d durations_phase_;  // durations in phase (0 to 1)
};

}  // namespace unitree_ros
