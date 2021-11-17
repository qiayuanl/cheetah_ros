//
// Created by qiayuan on 2021/11/17.
//

#pragma once
#include "mpc_formulation.h"
#include <mutex>
#include <thread>
#include <qpOASES.hpp>

namespace unitree_ros
{
class MpcSolverBase
{
public:
  ~MpcSolverBase()
  {
    delete thread_;
  }

protected:
  void solvingThread()
  {
    std::lock_guard<std::mutex> guard(mutex_);
  }
  MpcFormulation mpc_formulation_;

  std::mutex mutex_;
  std::thread* thread_;
};

class QpOasesSolver : public MpcSolverBase
{
public:
private:
};

}  // namespace unitree_ros
