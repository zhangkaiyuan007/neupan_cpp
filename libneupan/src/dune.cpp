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

#include "neupan/dune.hpp"

#include <algorithm>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace neupan {

DUNE::DUNE(const Robot& robot, const std::string& checkpoint_path)
    : mlp_(MLP::load(checkpoint_path)), G_(robot.G), h_(robot.h), T_(robot.T) {
  if (mlp_.outputDim() != G_.rows())
    throw std::runtime_error("dune: checkpoint edge_dim mismatch with robot G");
}

DUNE::Output DUNE::forward(const std::vector<Mat2X>& point_flow,
                           const std::vector<Mat22>& R_list,
                           const std::vector<Mat2X>& obs_points_list) const {
  const size_t stages = point_flow.size();  // T+1
  if (R_list.size() != stages || obs_points_list.size() != stages)
    throw std::invalid_argument("dune: list size mismatch");

  Eigen::Index total = 0;
  for (const auto& p : point_flow) total += p.cols();

  // Batched MLP over all stages in float32, as upstream.
  MatF all_points(2, total);
  Eigen::Index off = 0;
  for (const auto& p : point_flow) {
    all_points.middleCols(off, p.cols()) = p.cast<float>();
    off += p.cols();
  }
  const MatF total_mu = mlp_.forward(all_points);

  Output out;
  out.mu_list.reserve(stages);
  out.lam_list.reserve(stages);
  out.sort_point_list.reserve(stages);
  out.min_distance = std::numeric_limits<double>::infinity();

  off = 0;
  for (size_t i = 0; i < stages; ++i) {
    const Eigen::Index n = point_flow[i].cols();
    const Mat mu = total_mu.middleCols(off, n).cast<double>();
    off += n;

    const Mat2X lam = -R_list[i] * G_.transpose() * mu;

    // distance_k = mu_k . (G p0_k - h)
    Vec distance(n);
    const Mat gp = (G_ * point_flow[i]).colwise() - h_;
    for (Eigen::Index k = 0; k < n; ++k)
      distance(k) = mu.col(k).dot(gp.col(k));

    if (i == 0 && n > 0) out.min_distance = distance.minCoeff();

    std::vector<Eigen::Index> idx(n);
    std::iota(idx.begin(), idx.end(), 0);
    std::stable_sort(idx.begin(), idx.end(), [&](Eigen::Index a, Eigen::Index b) {
      return distance(a) < distance(b);
    });

    Mat mu_sorted(mu.rows(), n);
    Mat2X lam_sorted(2, n);
    Mat2X pts_sorted(2, n);
    for (Eigen::Index k = 0; k < n; ++k) {
      mu_sorted.col(k) = mu.col(idx[k]);
      lam_sorted.col(k) = lam.col(idx[k]);
      pts_sorted.col(k) = obs_points_list[i].col(idx[k]);
    }

    out.mu_list.push_back(std::move(mu_sorted));
    out.lam_list.push_back(std::move(lam_sorted));
    out.sort_point_list.push_back(std::move(pts_sorted));
  }

  return out;
}

}  // namespace neupan
