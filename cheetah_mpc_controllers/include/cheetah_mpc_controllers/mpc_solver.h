//
// Created by qiayuan on 2021/11/17.
//

#pragma once
#include "mpc_formulation.h"
#include <mutex>
#include <thread>
#include <qpOASES.hpp>
#include <ros/ros.h>

namespace cheetah_ros
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

  void setup(double dt, int horizon, double f_max, const Matrix<double, 13, 1>& weight, double alpha)
  {
    dt_ = dt;
    f_max_ = f_max;
    weight_ = weight;
    alpha_ = alpha;
    horizon_ = horizon;
    mpc_formulation_.setup(horizon, weight, alpha);
  }

  void setHorizon(int horizon)
  {
    horizon_ = horizon;
  }

  void solve(ros::Time time, const RobotState& state, const VectorXd& gait_table, const Matrix<double, Dynamic, 1>& traj)
  {
    double dt = (time - last_update_).toSec();

    if (dt < 0)  // Simulation reset
      last_update_ = time;
    if (dt > dt_)
    {
      std::unique_lock<std::mutex> guard(mutex_, std::try_to_lock);
      if (guard.owns_lock())
      {
        if (horizon_ != mpc_formulation_.horizon_)
          setup(dt_, horizon_, f_max_, weight_, alpha_);

        last_update_ = time;
        state_ = state;
        gait_table_ = gait_table;
        traj_ = traj;

        thread_ = std::make_shared<std::thread>(std::thread(&MpcSolverBase::solvingThread, this));
        thread_->detach();
      }
      else
        ROS_WARN("Solve timeout.");
    }
  }

  const std::vector<Vec3<double>>& getSolution()
  {
    return solution_;
  }

  int getHorizon()
  {
    return horizon_;
  }

  double getDt()
  {
    return dt_;
  };

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
  int horizon_;

private:
  void formulate()
  {
    mpc_formulation_.buildStateSpace(mass_, inertia_, state_);
    mpc_formulation_.buildQp(dt_);
    mpc_formulation_.buildHessianMat();
    mpc_formulation_.buildGVec(gravity_, state_, traj_);
    mpc_formulation_.buildConstrainMat(mu_);
    mpc_formulation_.buildConstrainUpperBound(f_max_, gait_table_);
    mpc_formulation_.buildConstrainLowerBound();
    ROS_INFO_STREAM(traj_);
    ROS_INFO_STREAM(getHorizon());
    mpc_formulation_.buildStateUpperBound(traj_.block<12, 1>(12 * (mpc_formulation_.horizon_ - 1), 0));
    mpc_formulation_.buildStateLowerBound(traj_.block<12, 1>(12 * (getHorizon() - 1), 0));
  }

  ros::Time last_update_;

  double dt_, mass_, gravity_, mu_, f_max_;
  Matrix3d inertia_;
  RobotState state_;
  Matrix<double, Dynamic, 1> traj_;
  VectorXd gait_table_;

  // Only for setHorizon()
  Matrix<double, 13, 1> weight_;
  double alpha_;
};

class QpOasesSolver : public MpcSolverBase
{
public:
  using MpcSolverBase::MpcSolverBase;

protected:
  void solving() override
  {
    auto qp_problem =
        qpOASES::QProblem(12 * mpc_formulation_.horizon_, 20 * mpc_formulation_.horizon_);  // TODO: Test SQProblem
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_NONE;
    qp_problem.setOptions(options);
    int n_wsr = 100;
    qpOASES::returnValue rvalue =
        qp_problem.init(mpc_formulation_.h_.data(), mpc_formulation_.g_.data(), mpc_formulation_.a_.data(),
                        mpc_formulation_.lb_.data(), mpc_formulation_.ub_.data(), mpc_formulation_.lb_a_.data(),
                        mpc_formulation_.ub_a_.data(), n_wsr);
    printFailedInit(rvalue);

    if (rvalue != qpOASES::SUCCESSFUL_RETURN)
    {
      for (auto& solution : solution_)
        solution.setZero();
      return;
    }

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

  void printFailedInit(qpOASES::returnValue rvalue)
  {
    switch (rvalue)
    {
      case qpOASES::RET_INIT_FAILED:
        ROS_WARN("MPC init failed");
        break;
      case qpOASES::RET_INIT_FAILED_CHOLESKY:
        ROS_WARN("MPC init failed with: RET_INIT_FAILED_CHOLESKY");
        break;
      case qpOASES::RET_INIT_FAILED_TQ:
        ROS_WARN("MPC init failed with: RET_INIT_FAILED_CHOLESKY");
        break;
      case qpOASES::RET_INIT_FAILED_HOTSTART:
        ROS_WARN("MPC init failed: RET_INIT_FAILED_HOTSTART");
        break;
      case qpOASES::RET_INIT_FAILED_INFEASIBILITY:
        ROS_WARN("MPC init failed: RET_INIT_FAILED_INFEASIBILITY");
        break;
      case qpOASES::RET_INIT_FAILED_UNBOUNDEDNESS:
        ROS_WARN("MPC init failed: RET_INIT_FAILED_UNBOUNDEDNESS");
        break;
      case qpOASES::RET_MAX_NWSR_REACHED:
        ROS_WARN("MPC init failed: RET_MAX_NWSR_REACHED");
        break;
      case qpOASES::RET_INVALID_ARGUMENTS:
        ROS_WARN("MPC init failed: RET_INVALID_ARGUMENTS");
        break;
      default:
        break;
    }
  }
};

}  // namespace cheetah_ros
