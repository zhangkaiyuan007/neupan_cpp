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
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <optional>

#include "neupan/neupan_planner.hpp"

using namespace neupan;

namespace {
const std::string kData = NEUPAN_TEST_DATA_DIR;
const std::string kModel = std::string(NEUPAN_MODEL_DIR) + "/diff_default.bin";

struct Outcome {
  Vec2 first_action;
  double miss = 0.0;  // final distance to the goal
  double min_distance = 1e9;
  double max_change = 0.0;  // largest change of the action from one step to the next
  double acce_bound = 0.0;  // max_acce * dt
};

// Closed loop from rest at the origin to goal, with the omni replay config;
// with continuous, each step starts from the previous action.
Outcome drive(const Vec2& goal, const Mat2X& points, bool continuous = false) {
  NeuPANPlanner planner =
      NeuPANPlanner::fromYaml(kData + "/replay_omni_planner.yaml", kModel);
  Vec3 s = Vec3::Zero();
  planner.updateInitialPathFromGoal(
      s, Vec3(goal.x(), goal.y(), std::atan2(goal.y(), goal.x())));

  Outcome run;
  run.acce_bound = planner.robot().acce_bound(0);
  Vec2 prev = Vec2::Zero();
  for (int k = 0; k < 60; ++k) {
    NeuPANPlanner::Info info;
    std::optional<NeuPANPlanner::Command> command;
    if (continuous) command = NeuPANPlanner::Command{prev, planner.robot().dt};
    const Vec2 a = planner.forward(s, points, info, command);
    if (k == 0) run.first_action = a;
    run.min_distance = std::min(run.min_distance, info.min_distance);
    run.max_change = std::max(run.max_change, (a - prev).norm());
    s.head<2>() += planner.robot().dt * a;
    prev = a;
  }
  // These weights (q_s 0.1, p_u 2) settle short of arrive_threshold.
  run.miss = (s.head<2>() - goal).norm();
  return run;
}
}  // namespace

// Upstream drives straight on along the zero nominal direction and never
// turns round; seedOmni starts it along the path.
TEST(Omni, StartsTowardsAGoalBehind) {
  const Outcome run = drive(Vec2(-8.0, 0.0), Mat2X(2, 0));
  EXPECT_LT(run.first_action.x(), -1.0);
  EXPECT_LT(run.miss, 0.3);
}

TEST(Omni, GoesRoundAWall) {
  Mat2X wall(2, 31);
  for (int i = 0; i < wall.cols(); ++i)
    wall.col(i) = Vec2(-4.0, -1.5 + 0.1 * i);
  const Outcome run = drive(Vec2(-8.0, 0.0), wall);
  EXPECT_LT(run.miss, 0.3);
  EXPECT_GT(run.min_distance, 0.0);
}

// Without a command the first step jumps to ref_speed. With one, the action
// changes by at most the corner of the (speed, direction) box round the disk
// of radius max_acce * dt, sqrt(5) times the radius.
TEST(Omni, CommandBoundsTheChange) {
  Mat2X wall(2, 31);
  for (int i = 0; i < wall.cols(); ++i)
    wall.col(i) = Vec2(-4.0, -1.5 + 0.1 * i);
  const Outcome free = drive(Vec2(-8.0, 0.0), wall);
  const Outcome run = drive(Vec2(-8.0, 0.0), wall, true);
  EXPECT_GT(free.max_change, 3.0 * free.acce_bound);
  EXPECT_LE(run.first_action.norm(), run.acce_bound + 1e-6);
  EXPECT_LE(run.max_change, std::sqrt(5.0) * run.acce_bound + 1e-6);
  EXPECT_LT(run.miss, 0.3);
  EXPECT_GT(run.min_distance, 0.0);
}

// Driving down a straight path from 5 cm to its left, between two of its
// points (ref_speed * dt = 0.4 m apart here), with the state and speed
// weights of neupan_cpp_ros/config/sentry_omni.yaml: the largest speed
// towards the left once back, i.e. the overshoot.
double overshoot(bool projected) {
  YAML::Node y = YAML::LoadFile(kData + "/replay_omni_planner.yaml");
  y["ipath"]["projected_reference"] = projected;
  y["adjust"]["q_s"] = 1.0;
  y["adjust"]["p_u"] = 1.0;
  const std::string yaml = testing::TempDir() + "/projected_reference.yaml";
  std::ofstream(yaml) << y;
  NeuPANPlanner planner = NeuPANPlanner::fromYaml(yaml, kModel);
  planner.setInitialPath({{0, 0, 0, 1}, {20, 0, 0, 1}});
  Vec3 s(0.15, 0.05, 0.0);
  double worst = 0.0;
  for (int k = 0; k < 30; ++k) {
    NeuPANPlanner::Info info;
    const Vec2 a = planner.forward(s, Mat2X(2, 0), info);
    worst = std::max(worst, a.y());
    s.head<2>() += planner.robot().dt * a;
  }
  return worst;
}

// Upstream's reference starts at the path point behind the robot, and the
// plan swings across the path and back.
TEST(Omni, ProjectedReferenceDoesNotOvershoot) {
  EXPECT_GT(overshoot(false), 0.3);
  EXPECT_LT(overshoot(true), 0.02);
}
