/*
 * neupan_cpp: C++ port of the NeuPAN planner.
 *
 * Ported from NeuPAN (https://github.com/hanruihua/NeuPAN),
 * Copyright (c) 2025 Ruihua Han <hanrh@connect.hku.hk>.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version. See <https://www.gnu.org/licenses/>.
 */

#include "neupan/robot.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace neupan {

namespace {

// Cross product sign sweep: returns +1 for CCW, -1 for CW, throws if the
// polygon is not convex (mirrors neupan.util.is_convex_and_ordered).
int convexOrientation(const Mat2X& v) {
  const Eigen::Index n = v.cols();
  if (n < 3) throw std::invalid_argument("robot: polygon needs >= 3 vertices");

  double direction = 0.0;
  for (Eigen::Index i = 0; i < n; ++i) {
    const Vec2 o = v.col(i);
    const Vec2 a = v.col((i + 1) % n);
    const Vec2 b = v.col((i + 2) % n);
    const double cross =
        (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
    if (cross != 0.0) {
      if (direction == 0.0) {
        direction = cross;
      } else if (direction * cross < 0.0) {
        throw std::invalid_argument("robot: polygon is not convex");
      }
    }
  }
  return direction > 0.0 ? 1 : -1;
}

}  // namespace

void genInequalFromVertex(const Mat2X& vertices, Mat& G, Vec& h) {
  Mat2X v = vertices;
  if (convexOrientation(v) < 0) {
    // CW input: keep first vertex, reverse the rest (as upstream does).
    Mat2X r(2, v.cols());
    r.col(0) = v.col(0);
    for (Eigen::Index i = 1; i < v.cols(); ++i)
      r.col(i) = v.col(v.cols() - i);
    v = r;
  }

  const Eigen::Index num = v.cols();
  G.resize(num, 2);
  h.resize(num);

  for (Eigen::Index i = 0; i < num; ++i) {
    const Vec2 pre = v.col(i);
    const Vec2 next = v.col((i + 1) % num);
    const Vec2 diff = next - pre;
    G(i, 0) = diff.y();
    G(i, 1) = -diff.x();
    h(i) = G(i, 0) * pre.x() + G(i, 1) * pre.y();
  }
}

Robot::Robot(Kinematics kinematics, int receding, double step_time,
             Vec2 max_speed_in, Vec2 max_acce, const Mat2X& vertices_in,
             double wheelbase_in)
    : kinematics(kinematics),
      T(receding),
      dt(step_time),
      wheelbase(wheelbase_in),
      max_speed(std::move(max_speed_in)),
      acce_bound(max_acce * step_time),
      vertices(vertices_in) {
  if (kinematics == Kinematics::Acker) {
    if (wheelbase <= 0.0)
      throw std::invalid_argument("robot: Ackermann wheelbase must be > 0");
    // Match the Python implementation's guard while staying strictly away
    // from tan(pi/2), where the linearization becomes singular.
    constexpr double kMaxSteeringAngle = 1.57;
    max_speed(1) = std::min(max_speed(1), kMaxSteeringAngle);
  }
  genInequalFromVertex(vertices, G, h);
}

Robot Robot::diffRectangle(int receding, double step_time, Vec2 max_speed,
                           Vec2 max_acce, double length, double width,
                           double wheelbase) {
  const double sx = -(length - wheelbase) / 2.0;
  const double sy = -width / 2.0;
  Mat2X v(2, 4);
  v.col(0) << sx, sy;
  v.col(1) << sx + length, sy;
  v.col(2) << sx + length, sy + width;
  v.col(3) << sx, sy + width;
  return Robot(Kinematics::Diff, receding, step_time, std::move(max_speed),
               std::move(max_acce), v, wheelbase);
}

Robot Robot::ackerRectangle(int receding, double step_time, Vec2 max_speed,
                            Vec2 max_acce, double length, double width,
                            double wheelbase) {
  const double sx = -(length - wheelbase) / 2.0;
  const double sy = -width / 2.0;
  Mat2X vertices(2, 4);
  vertices.col(0) << sx, sy;
  vertices.col(1) << sx + length, sy;
  vertices.col(2) << sx + length, sy + width;
  vertices.col(3) << sx, sy + width;
  return Robot(Kinematics::Acker, receding, step_time, std::move(max_speed),
               std::move(max_acce), vertices, wheelbase);
}

void Robot::linearize(const Vec3& nom_s_t, const Vec2& nom_u_t, Mat33& A,
                      Mat32& B, Vec3& C) const {
  const double phi = nom_s_t(2);
  const double v = nom_u_t(0);

  switch (kinematics) {
    case Kinematics::Diff:
      A << 1, 0, -v * dt * std::sin(phi),  //
          0, 1, v * dt * std::cos(phi),    //
          0, 0, 1;
      B << std::cos(phi) * dt, 0,  //
          std::sin(phi) * dt, 0,   //
          0, dt;
      C << phi * v * std::sin(phi) * dt, -phi * v * std::cos(phi) * dt, 0;
      break;
    case Kinematics::Acker: {
      const double psi = nom_u_t(1);
      const double sin_phi = std::sin(phi);
      const double cos_phi = std::cos(phi);
      const double cos_psi = std::cos(psi);
      const double inv_l = 1.0 / wheelbase;
      const double tan_psi = std::tan(psi);
      const double sec2_psi = 1.0 / (cos_psi * cos_psi);

      // Exact port of Python robot.linear_ackermann_model. Fixed-size Eigen
      // matrices keep this allocation-free on the per-stage hot path.
      A << 1, 0, -v * dt * sin_phi,  //
          0, 1, v * dt * cos_phi,   //
          0, 0, 1;
      B << cos_phi * dt, 0,  //
          sin_phi * dt, 0,   //
          tan_psi * dt * inv_l, v * dt * inv_l * sec2_psi;
      C << phi * v * sin_phi * dt, -phi * v * cos_phi * dt,
          -psi * v * dt * inv_l * sec2_psi;
      break;
    }
  }
}

}  // namespace neupan
