//
// Created by qiayuan on 2021/11/17.
//

#pragma once
#include "mpc_formulation.h"
#include <mutex>
#include <thread>
#include <qpOASES.hpp>
#include <ros/ros.h>

namespace unitree_ros
{
class MpcSolverBase
{
public:
  virtual ~MpcSolverBase(){};
  MpcSolverBase(double mass, double gravity, double mu, const Matrix3d& inertia)
    : mass_(mass), gravity_(gravity), mu_(mu), inertia_(inertia)
  {
    solution_.resize(4);
  }

  void setup(double dt, int horizon, double f_max, const Matrix<double, 13, 1>& weight)
  {
    std::lock_guard<std::mutex> guard(mutex_);
    dt_ = dt;
    f_max_ = f_max;
    mpc_formulation_.setup(horizon, weight);
  }

  void solve(ros::Time time, const RobotState& state, const VectorXd& gait_table, const Matrix<double, Dynamic, 1>& traj)
  {
    if ((time - last_update_).toSec() > dt_ && time > ros::Time(0))
    {
      std::unique_lock<std::mutex> guard(mutex_, std::try_to_lock);
      if (guard.owns_lock())
      {
        last_update_ = time;
        state_ = state;
        gait_table_ = gait_table;
        traj_ = traj;

        thread_ = std::make_shared<std::thread>(std::thread(&MpcSolverBase::solvingThread, this));
        thread_->detach();
      }
    }
  }

  const std::vector<Vec3<double>>& getSolution()
  {
    return solution_;
  }

protected:
  void solvingThread()
  {
    std::lock_guard<std::mutex> guard(mutex_);
    formulate();
    solving();
  };

  virtual void solving() = 0;

  MpcFormulation mpc_formulation_;
  std::vector<Vec3<double>> solution_;

  std::mutex mutex_;
  std::shared_ptr<std::thread> thread_;

private:
  void formulate()
  {
    mpc_formulation_.buildStateSpace(mass_, inertia_, state_);
    mpc_formulation_.buildQp(dt_);
    mpc_formulation_.buildHessianMat();
    mpc_formulation_.buildGVec(gravity_, state_, traj_);
    mpc_formulation_.buildConstrainMat(mu_);
    mpc_formulation_.buildUpperBound(f_max_, gait_table_);
    mpc_formulation_.buildLowerBound();
  }

  ros::Time last_update_;

  double dt_, mass_, gravity_, mu_, f_max_;
  Matrix3d inertia_;
  RobotState state_;
  Matrix<double, Dynamic, 1> traj_;
  VectorXd gait_table_;
};

class QpOasesSolver : public MpcSolverBase
{
public:
  using MpcSolverBase::MpcSolverBase;

protected:
  void solving() override
  {
    auto qp_problem = qpOASES::QProblem(12 * mpc_formulation_.horizon_, 20 * mpc_formulation_.horizon_);
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_LOW;
    qp_problem.setOptions(options);
    int n_wsr = 1000;
    qp_problem.init(mpc_formulation_.h_.data(), mpc_formulation_.g_.data(), mpc_formulation_.c_.data(), nullptr,
                    nullptr, mpc_formulation_.l_b_.data(), mpc_formulation_.u_b_.data(), n_wsr);
    std::vector<qpOASES::real_t> qp_sol(12 * mpc_formulation_.horizon_, 0);

    if (qp_problem.getPrimalSolution(qp_sol.data()) != qpOASES::SUCCESSFUL_RETURN)
      ROS_WARN("Failed to solve mpc!\n");

    for (int leg = 0; leg < 4; ++leg)
    {
      solution_[leg].x() = qp_sol[3 * leg];
      solution_[leg].y() = qp_sol[3 * leg + 1];
      solution_[leg].z() = qp_sol[3 * leg + 2];
    }
  }
};

}  // namespace unitree_ros
