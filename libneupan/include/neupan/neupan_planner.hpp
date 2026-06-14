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

#pragma once

#include <memory>
#include <string>

#include "neupan/initial_path.hpp"
#include "neupan/pan.hpp"
#include "neupan/robot.hpp"
#include "neupan/types.hpp"

namespace neupan {

// Top-level planner API, port of neupan/neupan.py.
class NeuPANPlanner {
 public:
  struct Config {
    int receding = 10;
    double step_time = 0.1;
    double ref_speed = 4.0;
    double collision_threshold = 0.1;

    // robot (diff rectangle)
    Vec2 max_speed{8.0, 1.0};
    Vec2 max_acce{8.0, 3.0};
    double length = 1.6;
    double width = 2.0;
    double wheelbase = 0.0;

    InitialPath::Options ipath;
    PAN::Options pan;
    NRMPParams adjust;

    std::string dune_checkpoint;  // NPTF .bin model path
  };

  explicit NeuPANPlanner(const Config& config);

  // Loads the upstream-compatible YAML (planner.yaml). `dune_checkpoint`
  // must point to an exported .bin model; the value can also be overridden
  // by the second argument.
  static NeuPANPlanner fromYaml(const std::string& yaml_path,
                                const std::string& checkpoint_override = "");

  struct Info {
    bool arrive = false;
    bool stop = false;
    double min_distance = 0.0;
    Mat3X opt_s;        // MPC trajectory
    Mat2X opt_u;        // full control sequence
    Mat3X ref_s;        // reference states
    Mat2X dune_points;  // visualization: points fed to DUNE
    Mat2X nrmp_points;  // visualization: closest points in NRMP
  };

  // state: (3,1) x,y,theta; points: (2,N) global frame (N may be 0).
  // Returns the action (2,1) and fills info.
  Vec2 forward(const Vec3& state, const Mat2X& points, Info& info);

  void reset();
  void setReferenceSpeed(double speed) { ref_speed_ = speed; }
  // Override the internal nominal control sequence (testing / bridging).
  void setCurVelArray(const Mat2X& u) { cur_vel_array_ = u; }
  void updateInitialPathFromGoal(const Vec3& start, const Vec3& goal) {
    ipath_.updateInitialPathFromGoal(start, goal);
  }
  void setWaypoints(const std::vector<Vec3>& wps) {
    ipath_.setIpathWithWaypoints(wps);
  }
  void setInitialPath(std::vector<InitialPath::PathPoint> path) {
    ipath_.setInitialPath(std::move(path));
  }
  double collisionThreshold() const { return cfg_.collision_threshold; }
  const Config& config() const { return cfg_; }

  InitialPath& ipath() { return ipath_; }
  PAN& pan() { return pan_; }
  const Robot& robot() const { return robot_; }

 private:
  Config cfg_;
  Robot robot_;
  InitialPath ipath_;
  PAN pan_;
  double ref_speed_;
  Mat2X cur_vel_array_;  // (2, T)
};

}  // namespace neupan
