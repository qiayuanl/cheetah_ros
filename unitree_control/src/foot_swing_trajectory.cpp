/*!
 * @file FootSwingTrajectory.cpp
 * @brief Utility to generate foot swing trajectories.
 *
 * Currently uses Bezier curves like Cheetah 3 does
 */

#include "unitree_control/foot_swing_trajectory.h"
#include "unitree_control/interpolation.h"

namespace unitree_ros
{
/*!
 * Compute foot swing trajectory with a bezier curve
 * @param phase : How far along we are in the swing (0 to 1)
 * @param swingTime : How long the swing should take (seconds)
 */
template <typename T>
void FootSwingTrajectory<T>::computeSwingTrajectoryBezier(T phase, T swingTime)
{
  p_ = interpolate::cubicBezier<Vec3<T>>(p0_, pf_, phase);
  v_ = interpolate::cubicBezierFirstDerivative<Vec3<T>>(p0_, pf_, phase) / swingTime;
  a_ = interpolate::cubicBezierSecondDerivative<Vec3<T>>(p0_, pf_, phase) / (swingTime * swingTime);

  T zp, zv, za;

  if (phase < T(0.5))
  {
    zp = interpolate::cubicBezier<T>(p0_[2], p0_[2] + height_, phase * 2);
    zv = interpolate::cubicBezierFirstDerivative<T>(p0_[2], p0_[2] + height_, phase * 2) * 2 / swingTime;
    za = interpolate::cubicBezierSecondDerivative<T>(p0_[2], p0_[2] + height_, phase * 2) * 4 / (swingTime * swingTime);
  }
  else
  {
    zp = interpolate::cubicBezier<T>(p0_[2] + height_, pf_[2], phase * 2 - 1);
    zv = interpolate::cubicBezierFirstDerivative<T>(p0_[2] + height_, pf_[2], phase * 2 - 1) * 2 / swingTime;
    za = interpolate::cubicBezierSecondDerivative<T>(p0_[2] + height_, pf_[2], phase * 2 - 1) * 4 /
         (swingTime * swingTime);
  }

  p_[2] = zp;
  v_[2] = zv;
  a_[2] = za;
}

template class FootSwingTrajectory<double>;
template class FootSwingTrajectory<float>;

}  // namespace unitree_ros
