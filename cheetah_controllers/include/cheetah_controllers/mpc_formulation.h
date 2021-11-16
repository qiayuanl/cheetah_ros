//
// Created by qiayuan on 2021/11/16.
//
#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <qpoases_vendor/qpOASES.hpp>

namespace unitree_ros
{
class MpcFormulation
{
public:
  struct Config
  {
    double dt_;
    double mu_;
    double f_max_;
    int horizon_;
  };

  MpcFormulation(Config setup);
};

}  // namespace unitree_ros
