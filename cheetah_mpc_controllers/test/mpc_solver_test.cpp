//
// Created by qiayuan on 2021/12/14.
//

#include <cheetah_mpc_controllers/mpc_solver.h>

using namespace std;
using namespace chrono;

using namespace cheetah_ros;
using namespace Eigen;

int main()
{
  int horizon = 1;

  double mass = 11.041;
  Matrix3d inertia;
  inertia << 0.050874, 0., 0., 0., 0.64036, 0., 0., 0., 0.6565;

  std::shared_ptr<MpcSolverBase> mpc_solver = std::make_shared<QpOasesSolver>(mass, -9.81, 0.3, inertia);
  Matrix<double, 13, 1> weight;
  weight << 0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2, 0.;

  mpc_solver->setup(0.01, horizon, 100., weight, 1e-6);

  // State space
  RobotState state;
  state.pos_ << 0, 0, 0.25;
  state.quat_.setIdentity();
  state.foot_pos_[0] << 0.25, 0.2, 0;
  state.foot_pos_[1] << 0.25, -0.2, 0;
  state.foot_pos_[2] << -0.25, 0.2, 0;
  state.foot_pos_[3] << -0.25, -0.2, 0;

  Eigen::VectorXd gait_table;
  gait_table.resize(horizon * 4);
  gait_table.setOnes();
  Eigen::VectorXd traj;
  traj.resize(12 * horizon);
  traj.setZero();
  for (int i = 0; i < horizon; ++i)
  {
    traj[12 * i + 3] = 0.2;
    traj[12 * i + 5] = 0.2;
  }
  mpc_solver->solve(ros::Time(0.1), state, gait_table, traj);
  sleep(1);
  for (const auto& force : mpc_solver->getSolution())
    std::cout << force << "\n" << std::endl;
  return 0;
}
