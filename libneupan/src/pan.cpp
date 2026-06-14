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

#include "neupan/pan.hpp"

#include <cmath>
#include <limits>

namespace neupan {

namespace {

// Port of neupan.util.downsample_decimation: uniform index sampling.
Mat2X downsampleDecimation(const Mat2X& mat, int m) {
  const int n = static_cast<int>(mat.cols());
  if (m >= n) return mat;
  Mat2X out(2, m);
  for (int i = 0; i < m; ++i) {
    const int idx = m == 1 ? 0
                           : static_cast<int>(static_cast<double>(i) *
                                              (n - 1) / (m - 1));
    out.col(i) = mat.col(idx);
  }
  return out;
}

}  // namespace

PAN::PAN(const Robot& robot, const std::string& dune_checkpoint,
         const NRMPParams& nrmp_params, const Options& opts)
    : robot_(robot),
      dune_(robot, dune_checkpoint),
      nrmp_(robot, nrmp_params),
      opts_(opts) {}

PAN::PointFlow PAN::generatePointFlow(const Mat3X& nom_s,
                                      const Mat2X& points) const {
  // Static obstacle points: upstream adds i * velocity * dt per stage,
  // point velocities are not ported yet (always zero in our scenarios).
  const Mat2X obs = downsampleDecimation(points, opts_.dune_max_num);

  PointFlow pf;
  pf.point_flow.reserve(robot_.T + 1);
  pf.obs_points.reserve(robot_.T + 1);
  pf.R.reserve(robot_.T + 1);

  for (int i = 0; i <= robot_.T; ++i) {
    const Vec2 trans = nom_s.col(i).head<2>();
    const double theta = nom_s(2, i);
    Mat22 R;
    R << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);

    pf.obs_points.push_back(obs);
    pf.point_flow.push_back(R.transpose() * (obs.colwise() - trans));
    pf.R.push_back(R);
  }
  return pf;
}

bool PAN::stopCriteria(const Mat3X& nom_s, const Mat2X& nom_u,
                       const DUNE::Output* dune_out) {
  if (!prev_) {
    prev_ = Prev{nom_s, nom_u, {}, {}};
    if (dune_out) {
      prev_->mu_list = dune_out->mu_list;
      prev_->lam_list = dune_out->lam_list;
    }
    return false;
  }

  double diff;
  if (!dune_out || prev_->mu_list.empty()) {
    diff = (nom_s - prev_->nom_s).squaredNorm() +
           (nom_u - prev_->nom_u).squaredNorm();
  } else {
    const int effect_num =
        std::min({static_cast<int>(dune_out->mu_list[0].cols()),
                  static_cast<int>(prev_->mu_list[0].cols()),
                  nrmp_.params().max_num});
    // upstream: norm(cat(mu_list)[:, :effect_num] - prev) / effect_num
    double mu_sq = 0.0, lam_sq = 0.0;
    for (size_t i = 0; i < dune_out->mu_list.size(); ++i) {
      mu_sq += (dune_out->mu_list[i].leftCols(effect_num) -
                prev_->mu_list[i].leftCols(effect_num))
                   .squaredNorm();
      lam_sq += (dune_out->lam_list[i].leftCols(effect_num) -
                 prev_->lam_list[i].leftCols(effect_num))
                    .squaredNorm();
    }
    diff = mu_sq / (static_cast<double>(effect_num) * effect_num) +
           lam_sq / (static_cast<double>(effect_num) * effect_num);
  }

  prev_ = Prev{nom_s, nom_u, {}, {}};
  if (dune_out) {
    prev_->mu_list = dune_out->mu_list;
    prev_->lam_list = dune_out->lam_list;
  }
  return diff < opts_.iter_threshold;
}

PAN::Output PAN::forward(Mat3X nom_s, Mat2X nom_u, const Mat3X& ref_s,
                         const Vec& ref_us, const Mat2X& points) {
  Output out;
  out.min_distance = std::numeric_limits<double>::infinity();
  // Bounded fallback: the nominal reference controls. If the very first NRMP
  // solve fails to converge we keep these rather than propagating a garbage
  // (constraint-violating) iterate from OSQP.
  out.opt_s = nom_s;
  out.opt_u = nom_u;

  for (int i = 0; i < opts_.iter_num; ++i) {
    std::vector<Mat> fa_list;
    std::vector<Vec> fb_list;
    std::optional<DUNE::Output> dune_out;

    if (points.cols() > 0) {
      const PointFlow pf = generatePointFlow(nom_s, points);
      dune_out = dune_.forward(pf.point_flow, pf.R, pf.obs_points);
      out.min_distance = dune_out->min_distance;
      out.dune_points = pf.obs_points[0];
      nrmp_.buildFaFb(dune_out->mu_list, dune_out->lam_list,
                      dune_out->sort_point_list, fa_list, fb_list);
      out.nrmp_points = dune_out->sort_point_list[0].leftCols(
          std::min<int>(nrmp_.params().max_num,
                        static_cast<int>(dune_out->sort_point_list[0].cols())));
    } else {
      nrmp_.buildFaFb({}, {}, {}, fa_list, fb_list);
    }

    const NRMP::Result res =
        nrmp_.solve(nom_s, nom_u, ref_s, ref_us, fa_list, fb_list);

    // A non-converged OSQP solve can return a point that violates the hard
    // speed/acceleration box; never propagate it. Keep the last accepted plan
    // (or the nominal fallback on the first iteration) instead.
    if (!res.success) break;

    nom_s = res.s;
    nom_u = res.u;
    out.opt_s = res.s;
    out.opt_u = res.u;
    out.opt_d = res.d;

    if (stopCriteria(nom_s, nom_u, dune_out ? &*dune_out : nullptr)) break;
  }
  return out;
}

}  // namespace neupan
