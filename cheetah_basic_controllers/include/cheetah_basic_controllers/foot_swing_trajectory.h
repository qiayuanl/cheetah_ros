/*!
 * @file FootSwingTrajectory.h
 * @brief Utility to generate foot swing trajectories.
 *
 * Currently uses Bezier curves like Cheetah 3 does
 */
#pragma once

#include <cheetah_common/cpp_types.h>
#include "interpolation.h"

namespace cheetah_ros
{
/*!
 * A foot swing trajectory for a single foot
 */
template <typename T>
class FootSwingTrajectory
{
public:
  /*!
   * Construct a new foot swing trajectory with everything set to zero
   */
  FootSwingTrajectory()
  {
    p0_.setZero();
    pf_.setZero();
    p_.setZero();
    v_.setZero();
    a_.setZero();
    height_ = 0;
  }

  /*!
   * Set the starting location of the foot
   * @param p0 : the initial foot position
   */
  void setInitialPosition(Vec3<T> p0)
  {
    p0_ = p0;
  }

  /*!
   * Set the desired final position of the foot
   * @param pf : the final foot position
   */
  void setFinalPosition(Vec3<T> pf)
  {
    pf_ = pf;
  }

  /*!
   * Set the maximum height of the swing
   * @param h : the maximum height of the swing, achieved halfway through the swing
   */
  void setHeight(T h)
  {
    height_ = h;
  }

  void computeSwingTrajectoryBezier(T phase, T swing_time);

  /*!
   * Get the foot position at the current point along the swing
   * @return : the foot position
   */
  Vec3<T> getPosition()
  {
    return p_;
  }

  /*!
   * Get the foot velocity at the current point along the swing
   * @return : the foot velocity
   */
  Vec3<T> getVelocity()
  {
    return v_;
  }

  /*!
   * Get the foot acceleration at the current point along the swing
   * @return : the foot acceleration
   */
  Vec3<T> getAcceleration()
  {
    return a_;
  }

private:
  Vec3<T> p0_, pf_, p_, v_, a_;
  T height_;
};

}  // namespace cheetah_ros
