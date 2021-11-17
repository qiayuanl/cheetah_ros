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

  struct Config
  {
    double dt_;
    double gravity_;
    double mu_;
    double mass_;
    double f_max_;
    int horizon_;
    Matrix3d inertia_;
    double weight_[12];
  };

  void setConfig(const Config& config);
  void updateModel(const RobotState& state);

  const MatrixXd& getHessianMat();
  const VectorXd& getGVec(const RobotState& state, const Matrix<double, Dynamic, 1>& traj);
  const MatrixXd& getConstrainMat();
  const VectorXd& getUpperBound(const RobotState& state);
  const VectorXd& getLowerBound();

private:
  void buildStateSpace(const Matrix3d& rot_yaw, const Matrix<double, 3, 4>& r_feet,
                       Matrix<double, STATE_DIM, STATE_DIM>& a, Matrix<double, STATE_DIM, ACTION_DIM>& b);
  void buildQp(const Matrix<double, STATE_DIM, STATE_DIM>& a_c, const Matrix<double, STATE_DIM, ACTION_DIM>& b_c,
               Matrix<double, Dynamic, STATE_DIM>& a_qp, MatrixXd& b_qp);

  // State Space Model
  Matrix<double, STATE_DIM, STATE_DIM> a_c_;
  Matrix<double, STATE_DIM, ACTION_DIM> b_c_;
  Matrix<double, Dynamic, STATE_DIM> a_qp_;
  MatrixXd b_qp_;

  // Weight
  // L matrix: Diagonal matrix of weights for state deviations
  Eigen::DiagonalMatrix<double, Eigen::Dynamic, Eigen::Dynamic> l_;

  // Final QP Formation
  // 1/2 U^{-T} H U + U^{T} g
  MatrixXd h_;    // hessian Matrix
  VectorXd g_;    // g vector
  MatrixXd c_;    // constrain matrix
  VectorXd u_b_;  // upper bound
  VectorXd l_b_;  // lower bound

  Config config_;
};

}  // namespace unitree_ros
