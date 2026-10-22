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

#include <stdexcept>

#include "neupan/neupan_planner.hpp"

using namespace neupan;

TEST(Config, RejectsContradictoryMargins) {
  NeuPANPlanner::Config c;
  EXPECT_NO_THROW(NeuPANPlanner::validate(c));

  NeuPANPlanner::Config bad = c;
  bad.adjust.d_max = 0.0;  // with eta > 0
  EXPECT_THROW(NeuPANPlanner::validate(bad), std::runtime_error);

  bad = c;
  bad.adjust.d_max = bad.adjust.d_min / 2;
  EXPECT_THROW(NeuPANPlanner::validate(bad), std::runtime_error);

  bad = c;
  bad.collision_threshold = 0.0;
  EXPECT_THROW(NeuPANPlanner::validate(bad), std::runtime_error);
}
