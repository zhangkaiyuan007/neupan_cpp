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

#include <gtest/gtest.h>

#include <cstdio>

#include "neupan/nrmp.hpp"
#include "neupan/robot.hpp"
#include "neupan/tensor_io.hpp"

using namespace neupan;

namespace {
const std::string kData = NEUPAN_TEST_DATA_DIR;
}

// Replays every dumped NRMP call (each frame, each PAM iteration) against
// the cvxpy/ECOS reference. M1 acceptance: action inf-norm diff < 5%,
// objective relative diff < 1%.
TEST(Nrmp, SolveMatchesCvxpy) {
  const TensorFile tf = TensorFile::load(kData + "/frames.nptf");

  const int T = static_cast<int>(tf.at("meta.T")(0, 0));
  const int n_frames = static_cast<int>(tf.at("meta.n_frames")(0, 0));

  const Robot robot =
      Robot::diffRectangle(T, tf.at("meta.dt")(0, 0), Vec2(8, 1), Vec2(8, 3),
                           1.6, 2.0);

  NRMPParams params;
  params.max_num = static_cast<int>(tf.at("meta.max_num")(0, 0));
  params.q_s = tf.at("meta.q_s").col(0);
  params.p_u = tf.at("meta.p_u")(0, 0);
  params.eta = tf.at("meta.eta")(0, 0);
  params.d_max = tf.at("meta.d_max")(0, 0);
  params.d_min = tf.at("meta.d_min")(0, 0);
  params.ro_obs = tf.at("meta.ro_obs")(0, 0);
  params.bk = tf.at("meta.bk")(0, 0);

  NRMP nrmp(robot, params);

  double max_action_diff = 0.0, max_obj_rel = 0.0, max_u0_scale = 0.0;
  int n_solves = 0;

  for (int f = 0; f < n_frames; ++f) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "f%02d", f);
    const int n_iters =
        static_cast<int>(tf.at(std::string(buf) + ".n_iters")(0, 0));

    for (int it = 0; it < n_iters; ++it) {
      std::snprintf(buf, sizeof(buf), "f%02d_it%d", f, it);
      const std::string pre = buf;

      const Mat3X nom_s = tf.at(pre + ".nom_s");
      const Mat2X nom_u = tf.at(pre + ".nom_u");
      const Mat3X ref_s = tf.at(pre + ".ref_s");
      const Vec ref_us = tf.at(pre + ".ref_us").col(0);

      // Rebuild fa/fb from the dumped (sorted) DUNE outputs, exercising
      // buildFaFb the same way the upstream NRMP consumes mu/lam/points.
      std::vector<Mat> mu_list;
      std::vector<Mat2X> lam_list, sp_list;
      for (int t = 0; t <= T; ++t) {
        std::snprintf(buf, sizeof(buf), "_s%02d", t);
        mu_list.push_back(tf.at(pre + ".mu" + buf));
        lam_list.push_back(tf.at(pre + ".lam" + buf));
        sp_list.push_back(tf.at(pre + ".sp" + buf));
      }
      std::vector<Mat> fa_list;
      std::vector<Vec> fb_list;
      nrmp.buildFaFb(mu_list, lam_list, sp_list, fa_list, fb_list);

      const NRMP::Result res =
          nrmp.solve(nom_s, nom_u, ref_s, ref_us, fa_list, fb_list);
      ASSERT_TRUE(res.success) << pre;
      ++n_solves;

      const Mat3X s_py = tf.at(pre + ".opt_s");
      const Mat2X u_py = tf.at(pre + ".opt_u");
      const Vec d_py = tf.at(pre + ".opt_d").row(0).transpose();

      // action = first control column
      const double action_diff =
          (res.u.col(0) - u_py.col(0)).cwiseAbs().maxCoeff();
      max_action_diff = std::max(max_action_diff, action_diff);
      max_u0_scale = std::max(max_u0_scale, u_py.col(0).cwiseAbs().maxCoeff());

      const double obj_cpp = res.objective;
      const double obj_py = nrmp.objective(s_py, u_py, d_py, nom_s, ref_s,
                                           ref_us, fa_list, fb_list);
      const double obj_rel =
          std::abs(obj_cpp - obj_py) / std::max(1.0, std::abs(obj_py));
      max_obj_rel = std::max(max_obj_rel, obj_rel);

      EXPECT_LT(action_diff, 0.05 * std::max(1.0, max_u0_scale)) << pre;
      EXPECT_LT(obj_rel, 0.01) << pre;
      // C++ optimum must not be worse than the reference solution.
      EXPECT_LT(obj_cpp, obj_py + 0.01 * std::max(1.0, std::abs(obj_py)))
          << pre;
    }
  }

  std::printf(
      "NRMP: %d solves, max action diff %.4f (scale %.3f), "
      "max objective rel diff %.3e\n",
      n_solves, max_action_diff, max_u0_scale, max_obj_rel);
}
