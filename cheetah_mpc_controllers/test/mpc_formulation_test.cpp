//
// Created by qiayuan on 2021/12/9.
//

#include <iostream>
#include <chrono>

#include <cheetah_mpc_controllers/mpc_formulation.h>

using namespace std;
using namespace chrono;

using namespace cheetah_ros;
using namespace Eigen;

int main()
{
  int horizon = 10;
  MpcFormulation mpc_formulation;

  Matrix<double, 13, 1> weight;
  weight << 0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2, 0.;
  mpc_formulation.setup(horizon, weight, 1e-6);

  // State space
  double mass = 11.041;
  Matrix3d inertia;
  inertia << 0.050874, 0., 0., 0., 0.64036, 0., 0., 0., 0.6565;

  RobotState state;
  state.pos_ << 0, 0, 0.25;
  state.quat_.setIdentity();
  state.foot_pos_[0] << 0.25, 0.2, 0;
  state.foot_pos_[1] << 0.25, -0.2, 0;
  state.foot_pos_[2] << -0.25, 0.2, 0;
  state.foot_pos_[3] << -0.25, -0.2, 0;

  auto start = system_clock::now();
  mpc_formulation.buildStateSpace(mass, inertia, state);  // Debug 0.001286s Release 0.0005s
  std::cout << "buildStateSpace spend "
            << double(duration_cast<microseconds>(system_clock::now() - start).count()) * microseconds::period::num /
                   microseconds::period::den
            << " second" << endl;

  start = system_clock::now();
  mpc_formulation.buildQp(0.003);
  std::cout << "buildQp spend "
            << double(duration_cast<microseconds>(system_clock::now() - start).count()) * microseconds::period::num /
                   microseconds::period::den
            << " second" << endl;

  start = system_clock::now();
  mpc_formulation.buildHessianMat();
  std::cout << "buildHessianMat spend "
            << double(duration_cast<microseconds>(system_clock::now() - start).count()) * microseconds::period::num /
                   microseconds::period::den
            << " second" << endl;

  Matrix<double, Dynamic, 1> traj;
  traj.resize(MpcFormulation::STATE_DIM * horizon);
  traj.setZero();
  start = system_clock::now();
  mpc_formulation.buildGVec(-9.81, state, traj);
  std::cout << "buildGVec spend "
            << double(duration_cast<microseconds>(system_clock::now() - start).count()) * microseconds::period::num /
                   microseconds::period::den
            << " second" << endl;

  return 0;
}
