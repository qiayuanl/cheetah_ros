//
// Created by qiayuan on 2021/12/14.
//

#include <cheetah_controllers/mpc_solver.h>

using namespace std;
using namespace chrono;

using namespace unitree_ros;
using namespace Eigen;

int main()
{
  int horizon = 1;

  double mass = 15.;
  Matrix3d inertia;
  inertia << 11253, 0, 0, 0, 36203, 0, 0, 0, 42673;

  std::shared_ptr<MpcSolverBase> mpc_solver = std::make_shared<QpOasesSolver>(mass, -9.81, 0.3, inertia);
  Matrix<double, 13, 1> weight;
  weight << 0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2, 0.;

  mpc_solver->setup(0.01, horizon, 100., weight);

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
    traj[12 * i + 2] = 0.2;

  mpc_solver->solve(ros::Time(0.1), state, gait_table, traj);

  while (1)
  {
    sleep(1);
  }
  return 0;
}
