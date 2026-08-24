/*
 * neupan_cpp: robot kinematics unit tests.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "neupan/robot.hpp"

using namespace neupan;

namespace {

Vec3 ackermannStep(const Vec3& s, const Vec2& u, double dt,
                   double wheelbase) {
  return s + dt * Vec3(u(0) * std::cos(s(2)), u(0) * std::sin(s(2)),
                       u(0) * std::tan(u(1)) / wheelbase);
}

}  // namespace

TEST(Robot, AckermannLinearizationMatchesNominalDynamics) {
  constexpr double dt = 0.1;
  constexpr double wheelbase = 2.7;
  const Robot robot = Robot::ackerRectangle(
      10, dt, Vec2(8.0, 0.6), Vec2(4.0, 0.8), 4.5, 1.8, wheelbase);
  const Vec3 state(1.2, -0.7, 0.35);
  const Vec2 control(3.4, -0.22);

  Mat33 A;
  Mat32 B;
  Vec3 C;
  robot.linearize(state, control, A, B, C);

  EXPECT_TRUE((A * state + B * control + C)
                  .isApprox(ackermannStep(state, control, dt, wheelbase),
                            1e-12));
}

TEST(Robot, AckermannLinearizationTracksSmallPerturbations) {
  constexpr double dt = 0.05;
  constexpr double wheelbase = 1.4;
  const Robot robot = Robot::ackerRectangle(
      8, dt, Vec2(3.0, 0.7), Vec2(2.0, 1.0), 2.0, 1.0, wheelbase);
  const Vec3 nominal_state(0.4, 1.1, -0.6);
  const Vec2 nominal_control(1.7, 0.25);

  Mat33 A;
  Mat32 B;
  Vec3 C;
  robot.linearize(nominal_state, nominal_control, A, B, C);

  const Vec3 state = nominal_state + Vec3(1e-4, -2e-4, 1e-4);
  const Vec2 control = nominal_control + Vec2(-1e-4, 2e-4);
  const Vec3 linearized = A * state + B * control + C;
  const Vec3 nonlinear = ackermannStep(state, control, dt, wheelbase);
  EXPECT_TRUE(linearized.isApprox(nonlinear, 1e-7));
}

TEST(Robot, AckermannRequiresPositiveWheelbase) {
  EXPECT_THROW(
      Robot::ackerRectangle(10, 0.1, Vec2(4.0, 0.5), Vec2(2.0, 1.0),
                            3.0, 1.5, 0.0),
      std::invalid_argument);
}

TEST(Robot, AckermannSteeringLimitMatchesPythonGuard) {
  const Robot robot = Robot::ackerRectangle(
      10, 0.1, Vec2(4.0, 2.0), Vec2(2.0, 1.0), 3.0, 1.5, 2.0);
  EXPECT_DOUBLE_EQ(robot.max_speed(1), 1.57);
}
