//
// Created by qiayuan on 2021/12/9.
//

#include <cheetah_controllers/mpc_formulation.h>

using namespace unitree_ros;
using namespace Eigen;

int main()
{
  RobotState state;
  Matrix3d inertia;
  Matrix<double, 13, 1> weight;

  MpcFormulation mpc_formulation;
  mpc_formulation.setUp(10, weight);

  mpc_formulation.buildStateSpace(5.0, inertia, state);
  return 0;
}
