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

#ifndef GZ_ARM_GZ_ARM_UTIL_GUARD_HPP
#define GZ_ARM_GZ_ARM_UTIL_GUARD_HPP

#include <tuple>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>

namespace gz_arm
{

/**
 * converts target end-effector velocities to limited joint velocities
 */
std::vector<double> endEffectorToJointVelocities(
  double vx, double vy, double vth,
  const std::vector<double> & thetas, double max_joint_v);

/**
 * computes camera x,y,theta from joint angles
 */
std::tuple<double, double, double> camFromJoints(const std::vector<double> & thetas);


/**
 * returns effective rotation of quaternion around just z-axis
 */
double angleAroundZAxis(const tf2::Quaternion & q);


/**
 * returns effective rotation of quaternion around just z-axis
 */
double angleAroundZAxisBasic(const tf2::Quaternion & q);

}

#endif // GZ_ARM_GZ_ARM_UTIL_GUARD_HPP
