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

#include <gtest/gtest.h>

// GZ Arm
#include "gz_arm/gz_arm_util.hpp"

TEST(gz_arm_util, cartesian_from_joints)
{
  const double max_joint_velocity = 100.0;
  const double dt = 1e-6;
  const double epsilon = 5e-4;
  struct Param
  {
    double j1, j2, j3;
    double vx, vy, vth;
  };
  // Don't let joint angles j2,j3 because it leads
  // to singularities that make it difficult/imposible to achieve
  // certain target end-effector velocites
  std::vector<Param> test_params{
    {0.1, -0.2, 2.0, 1.0, 0.0, -0.5},
    {1.0, 0.4, 3.0, 0.0, 1.0, 0.1},
    {0.4, -0.5, 3.7, 0.0, -0.5, -0.5}
  };
  for (unsigned idx = 0; idx < test_params.size(); ++idx) {
    auto [j1, j2, j3, vx, vy, vth] = test_params[idx];
    std::vector<double> jvels = gz_arm::endEffectorToJointVelocities(
      vx, vy, vth, {j1, j2, j3},
      max_joint_velocity);
    ASSERT_EQ(jvels.size(), 3);

    // Effective displacement of end-effector position should match target joint velocities
    auto [x1, y1, th1] = gz_arm::camFromJoints({j1, j2, j3});
    auto [x2, y2, th2] = gz_arm::camFromJoints(
      {j1 + jvels[0] * dt, j2 + jvels[1] * dt,
        j3 + jvels[2] * dt});
    EXPECT_NEAR((x2 - x1) / dt, vx, epsilon) << " idx " << idx;
    EXPECT_NEAR((y2 - y1) / dt, vy, epsilon) << " idx " << idx;
    EXPECT_NEAR((th2 - th1) / dt, vth, epsilon) << " idx " << idx;
  }
}


TEST(gz_arm_util, angle_around_z_axis)
{
  struct Param
  {
    double x, y, z;
  };

  // x,y,z are rotations around respective axis
  // x and y should be smallish rotations
  std::vector<Param> test_params{
    {0.0, 0.0, 1.0},
    {0.1, 0.0, -1.0},
    {0.0, -0.2, -0.5},
    {0.1, -0.1, -2.0},
    {0.1, -0.1, 0.0}
  };

  for (unsigned idx = 0; idx < test_params.size(); ++idx) {
    auto [x, y, z] = test_params[idx];

    tf2::Quaternion qx(tf2::Vector3(1, 0, 0), x);
    tf2::Quaternion qy(tf2::Vector3(0, 1, 0), y);
    tf2::Quaternion qz(tf2::Vector3(0, 0, 1), z);

    // Rotating around x and y axis will actually
    // change rotation around a z-axis a bit
    // so epsilon is pretty large
    tf2::Quaternion q(qx * qy * qz);

    const double epsilon = 2e-2;
    double z_angle = gz_arm::angleAroundZAxis(q);
    EXPECT_NEAR(z, z_angle, epsilon) << " idx " << idx;
    double z_angle_basic = gz_arm::angleAroundZAxisBasic(q);
    EXPECT_NEAR(z_angle, z_angle_basic, 1e-6) << " idx " << idx;
  }
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
