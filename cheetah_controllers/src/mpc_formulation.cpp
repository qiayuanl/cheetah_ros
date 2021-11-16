//
// Created by qiayuan on 2021/11/16.
//
// Refer: https://github.com/mit-biomimetics/Cheetah-Software
// Refer: https://github.com/google-research/motion_imitation

#include "cheetah_controllers/mpc_formulation.h"

#include <unsupported/Eigen/MatrixFunctions>

namespace unitree_ros
{
// Converts a vector to the skew symmetric matrix form. For an input vector
// [a, b, c], the output matrix would be:
//   [ 0, -c,  b]
//   [ c,  0, -a]
//   [-b,  a,  0]
Matrix3d convertToSkewSymmetric(const Vector3d& vec)
{
  Matrix3d skew_sym_mat;
  skew_sym_mat << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
  return skew_sym_mat;
}

void MpcFormulation::buildStateSpace(const Matrix3d& rot_yaw, const Matrix<double, 3, 4>& r_feet,
                                     Matrix<double, STATE_DIM, STATE_DIM>& a, Matrix<double, STATE_DIM, ACTION_DIM>& b)
{
  a.setZero();
  a.block<3, 3>(0, 6) = rot_yaw.transpose();

  a(3, 9) = 1.;
  a(4, 10) = 1.;
  a(5, 11) = 1.;
  a(11, 12) = 1.;

  //  b contains non_zero elements only in row 6 : 12.
  for (int i = 0; i < 4; ++i)
  {
    b.block<3, 3>(6, i * 3) = config_.inertia_.inverse() * convertToSkewSymmetric(r_feet.col(i));
    b.block(9, i * 3, 3, 3) = Matrix<double, 3, 3>::Identity() / config_.mass_;
  }
}

void MpcFormulation::buildQp(const Matrix<double, STATE_DIM, STATE_DIM>& a_c,
                             const Matrix<double, STATE_DIM, ACTION_DIM>& b_c, Matrix<double, Dynamic, STATE_DIM>& a_qp,
                             Matrix<double, Dynamic, Dynamic>& b_qp)
{
  // Convert model from continuous to discrete time
  Matrix<double, STATE_DIM + ACTION_DIM, STATE_DIM + ACTION_DIM> ab_c;
  ab_c.block(0, 0, STATE_DIM, STATE_DIM) = a_c;
  ab_c.block(0, STATE_DIM, STATE_DIM, ACTION_DIM) = b_c;
  ab_c = config_.dt_ * ab_c;
  Matrix<double, STATE_DIM + ACTION_DIM, STATE_DIM + ACTION_DIM> exp = ab_c.exp();
  Matrix<double, STATE_DIM, STATE_DIM> a_dt = exp.block(0, 0, STATE_DIM, STATE_DIM);
  Matrix<double, STATE_DIM, ACTION_DIM> b_dt = exp.block(0, STATE_DIM, STATE_DIM, ACTION_DIM);

  Matrix<double, STATE_DIM, STATE_DIM> power_mat[20];

  power_mat[0].setIdentity();
  for (int i = 1; i < config_.horizon_ + 1; i++)
    power_mat[i] = a_dt * power_mat[i - 1];

  for (int r = 0; r < config_.horizon_; r++)
  {
    a_qp.block(STATE_DIM * r, 0, STATE_DIM, STATE_DIM) = power_mat[r + 1];  // Adt.pow(r+1);
    for (int c = 0; c < config_.horizon_; c++)
    {
      if (r >= c)
      {
        int a_num = r - c;
        b_qp.block(STATE_DIM * r, ACTION_DIM * c, STATE_DIM, ACTION_DIM) = power_mat[a_num] * b_dt;
      }
    }
  }
}

void MpcFormulation::build(const StateEstimateBase::State& state, const Matrix3d& feet_pos,
                           const Matrix<double, Dynamic, 1>& traj)
{
  // Update x_0 and x_ref
  Vector3d rpy;
  rpy.setZero();
  Matrix<double, STATE_DIM, 1> x_0;
  Matrix<double, Dynamic, 1> x_ref;
  x_0 << rpy(2), rpy(1), rpy(0), state.pos_, state.angular_vel_, state.linear_vel_, -config_.gravity_;
  for (int i = 0; i < config_.horizon_; i++)
    for (int j = 0; j < STATE_DIM - 1; j++)
      x_ref(STATE_DIM * i + j, 0) = traj[12 * i + j];

  // Weight
  Matrix<double, STATE_DIM, 1> full_weight;
  for (int i = 0; i < STATE_DIM - 1; i++)
    full_weight(i) = config_.weight_[i];
  full_weight(12) = 0.f;
  Matrix<double, Dynamic, Dynamic> l;  // L matrix: Diagonal matrix of weights for state deviations
  l.diagonal() = full_weight.replicate(config_.horizon_, 1);

  // Calculate Hessian and g
  Matrix<double, STATE_DIM, STATE_DIM> a_c;
  Matrix<double, STATE_DIM, ACTION_DIM> b_c;
  Matrix<double, Dynamic, STATE_DIM> a_qp;
  Matrix<double, Dynamic, Dynamic> b_qp;

  Matrix3d rot;
  Matrix<double, 3, 4> r_feet;
  for (int i = 0; i < 4; ++i)
    r_feet.col(i) = feet_pos.col(i) - state.pos_;

  buildStateSpace(rot, r_feet, a_c, b_c);
  buildQp(a_c, b_c, a_qp, b_qp);

  h_ = 2. * (b_qp.transpose() * l * b_qp);  // TODO: add K
  g_ = 2. * b_qp.transpose() * l * (a_qp * x_0 - x_ref);
  // Calculate contains
  double mu_inv = 1.f / config_.mu_;
  Matrix<double, 5, 3> c_block;
  c_block << mu_inv, 0, 1.f, -mu_inv, 0, 1.f, 0, mu_inv, 1.f, 0, -mu_inv, 1.f, 0, 0, 1.f;
  for (int i = 0; i < config_.horizon_ * 4; i++)
    c_.block(i * 5, i * 3, 5, 3) = c_block;
  l_b_.setZero();
  for (int i = 0; i < config_.horizon_; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      const int row = (i * 4 + j) * 5;
      const double big_value = 1e10;
      u_b_(row) = big_value;
      u_b_(row + 1) = big_value;
      u_b_(row + 2) = big_value;
      u_b_(row + 3) = big_value;
      //      u_b_(row + 4) = config_.f_max_ * contact_state(i, j);
    }
  }
}

}  // namespace unitree_ros
