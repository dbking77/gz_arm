/*
Copyright (c) 2024 Derek King

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

// GZ Arm
#include "gz_arm/gz_arm_util.hpp"

// Ceres
#include "ceres/jet.h"

// Eigen
#include "Eigen/Dense"

// TF2
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace gz_arm
{

template<typename T>
void cartesianFromJoints(T & x, T & y, T & th, const T & j1, const T & j2, const T & j3)
{
  // TODO load from URDF or some parameters
  const double l1 = 0.3;
  const double l2 = 0.2;
  const double l3 = 0.15;
  using ceres::sin;
  using ceres::cos;
  x = cos(j1) * l1 + cos(j1 + j2) * l2 + cos(j1 + j2 + j3) * l3;
  y = sin(j1) * l1 + sin(j1 + j2) * l2 + sin(j1 + j2 + j3) * l3;
  th = j1 + j2 + j3;
}


std::tuple<double, double, double> camFromJoints(const std::vector<double> & thetas)
{
  double x, y, th;
  cartesianFromJoints<double>(x, y, th, thetas[0], thetas[1], thetas[2]);
  return std::make_tuple(x, y, th);
}


std::vector<double>
endEffectorToJointVelocities(
  double vx, double vy, double vth, const std::vector<double> & thetas,
  double max_joint_v)
{
  using Jet3 = ceres::Jet<double, 3>;
  Jet3 j1{thetas[0], 0};
  Jet3 j2{thetas[1], 1};
  Jet3 j3{thetas[2], 2};
  Jet3 x, y, th;
  cartesianFromJoints<Jet3>(x, y, th, j1, j2, j3);

  Eigen::Matrix<double, 3, 3> jacobian;
  jacobian.row(0) = x.v;
  jacobian.row(1) = y.v;
  jacobian.row(2) = th.v;

  Eigen::Vector<double, 3> target;
  target(0) = vx;
  target(1) = vy;
  target(2) = vth;

  // https://eigen.tuxfamily.org/dox-3.3/group__LeastSquares.html
  Eigen::Vector<double,
    3> joint_v = jacobian.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(target);

  // TODO : use Eigen functions?
  const double j1a = std::abs(joint_v(0));
  const double j2a = std::abs(joint_v(1));
  const double j3a = std::abs(joint_v(2));
  const double j_abs_max = std::max(j1a, std::max(j2a, j3a));

  double scale = 1.0;
  if (j_abs_max > max_joint_v) {
    scale = max_joint_v / j_abs_max;
  }

  joint_v *= scale;

  return {joint_v(0), joint_v(1), joint_v(2)};
}

double angleAroundZAxisBasic(const tf2::Quaternion & q)
{
  // use quaternion to rotate x-axis unit vector then determine effector rotation of vector around z-axis
  tf2::Vector3 v{tf2::quatRotate(q, tf2::Vector3(1, 0, 0))};
  return std::atan2(v.y(), v.x());
}

double angleAroundZAxis(const tf2::Quaternion & q)
{
  // For angleAroundZAxisBasic, the y and z input terms are zero,
  // and the z output is unused
  // this takes advantage of these to short-cut some calculations
  double d = q.length2();
  if (d == 0) {
    return 0.0;
  } else {
    double s = 2.0 / d;
    double ys = q.y() * s;
    double zs = q.z() * s;
    double yy = ys * q.y();
    double zz = zs * q.z();
    double xy = q.x() * ys;
    double wz = q.w() * zs;
    double x = 1.0 - (yy + zz);
    double y = xy + wz;
    return std::atan2(y, x);
  }
}

}  // namespace gz_arm
