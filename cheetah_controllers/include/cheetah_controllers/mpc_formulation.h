//
// Created by qiayuan on 2021/11/16.
//
#pragma once

#include <unitree_control/cpp_types.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace unitree_ros
{
using Eigen::Dynamic;
using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;

class MpcFormulation  // TODO: template for float
{
public:
  static constexpr int STATE_DIM = 13;   // 6 dof pose + 6 dof velocity + 1 gravity.
  static constexpr int ACTION_DIM = 12;  // 4 ground reaction force.

  void setUp(int horizon, const Matrix<double, STATE_DIM, 1>& weight);

  void buildStateSpace(double mass, const Matrix3d& inertia, const RobotState& state);
  void buildQp(double dt);

  const Matrix<double, Dynamic, Dynamic, Eigen::RowMajor>& buildHessianMat();
  const VectorXd& buildGVec(double gravity, const RobotState& state, const Matrix<double, Dynamic, 1>& traj);
  const Matrix<double, Dynamic, Dynamic, Eigen::RowMajor>& buildConstrainMat(double mu);
  const VectorXd& buildUpperBound(double f_max, const VectorXd& gait_table);
  const VectorXd& buildLowerBound();

  int horizon_;

  // Final QP Formation
  // 1/2 U^{-T} H U + U^{T} g
  Matrix<double, Dynamic, Dynamic, Eigen::RowMajor> h_;  // hessian Matrix
  VectorXd g_;                                           // g vector
  Matrix<double, Dynamic, Dynamic, Eigen::RowMajor> c_;  // constrain matrix
  VectorXd u_b_;                                         // upper bound
  VectorXd l_b_;                                         // lower bound

private:
  // State Space Model
  Matrix<double, STATE_DIM, STATE_DIM> a_c_;
  Matrix<double, STATE_DIM, ACTION_DIM> b_c_;
  Matrix<double, Dynamic, STATE_DIM> a_qp_;
  MatrixXd b_qp_;

  // Weight
  // L matrix: Diagonal matrix of weights for state deviations
  Eigen::DiagonalMatrix<double, Eigen::Dynamic, Eigen::Dynamic> l_;
};

}  // namespace unitree_ros
