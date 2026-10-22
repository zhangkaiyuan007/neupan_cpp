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

#include "neupan/neupan_planner.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace neupan {

namespace {

Robot makeRobot(const NeuPANPlanner::Config& c) {
  if (c.vertices.cols() > 0)
    return Robot(c.kinematics, c.receding, c.step_time, c.max_speed,
                 c.max_acce, c.vertices);
  return Robot::rectangle(c.kinematics, c.receding, c.step_time, c.max_speed,
                          c.max_acce, c.length, c.width, c.wheelbase);
}

}  // namespace

void NeuPANPlanner::validate(const Config& c) {
  const NRMPParams& a = c.adjust;
  const char* bad = nullptr;
  if (c.receding < 1 || c.step_time <= 0.0)
    bad = "receding must be >= 1 and step_time > 0";
  else if (a.d_min < 0.0)
    bad = "d_min < 0 lets the plan into obstacles";
  else if (a.d_max < a.d_min)
    bad = "d_max < d_min";
  else if (a.eta > 0.0 && a.d_max <= 0.0)
    bad = "d_max <= 0 pins the clearance to zero, so eta has no effect";
  else if (c.collision_threshold <= 0.0)
    bad = "collision_threshold must be > 0";
  if (bad) throw std::runtime_error(std::string("neupan config: ") + bad);
}

NeuPANPlanner::NeuPANPlanner(const Config& config)
    : cfg_(config),
      robot_(makeRobot(config)),
      ipath_(robot_, config.ref_speed, config.ipath),
      pan_(robot_, config.dune_checkpoint, config.adjust, config.pan),
      ref_speed_(config.ref_speed),
      cur_vel_array_(Mat2X::Zero(2, config.receding)) {
  validate(config);
}

NeuPANPlanner NeuPANPlanner::fromYaml(const std::string& yaml_path,
                                      const std::string& checkpoint_override) {
  const YAML::Node y = YAML::LoadFile(yaml_path);
  Config c;

  c.receding = y["receding"].as<int>(c.receding);
  c.step_time = y["step_time"].as<double>(c.step_time);
  c.ref_speed = y["ref_speed"].as<double>(c.ref_speed);
  c.collision_threshold =
      y["collision_threshold"].as<double>(c.collision_threshold);

  if (const auto r = y["robot"]) {
    const auto kin = r["kinematics"].as<std::string>("diff");
    if (kin == "diff")
      c.kinematics = Kinematics::Diff;
    else if (kin == "omni")
      c.kinematics = Kinematics::Omni;
    else
      throw std::runtime_error("neupan_cpp supports 'diff' and 'omni' "
                               "kinematics (got '" + kin + "')");
    if (r["max_speed"]) {
      const auto v = r["max_speed"].as<std::vector<double>>();
      c.max_speed = Vec2(v.at(0), v.at(1));
    }
    if (r["max_acce"]) {
      const auto v = r["max_acce"].as<std::vector<double>>();
      c.max_acce = Vec2(v.at(0), v.at(1));
    }
    c.length = r["length"].as<double>(c.length);
    c.width = r["width"].as<double>(c.width);
    c.wheelbase = r["wheelbase"].as<double>(0.0);
    if (r["vertices"]) {
      const auto v = r["vertices"].as<std::vector<std::vector<double>>>();
      c.vertices.resize(2, static_cast<int>(v.size()));
      for (size_t i = 0; i < v.size(); ++i)
        c.vertices.col(static_cast<int>(i)) = Vec2(v[i].at(0), v[i].at(1));
    }
  }

  if (const auto ip = y["ipath"]) {
    const auto style = ip["curve_style"].as<std::string>("line");
    if (style != "line")
      throw std::runtime_error("neupan_cpp currently supports only the "
                               "'line' curve_style (got '" + style + "')");
    if (ip["waypoints"])
      for (const auto& w : ip["waypoints"]) {
        const auto v = w.as<std::vector<double>>();
        c.ipath.waypoints.emplace_back(v.at(0), v.at(1), v.at(2));
      }
    c.ipath.loop = ip["loop"].as<bool>(false);
    c.ipath.interval = ip["interval"].as<double>(-1.0);
    c.ipath.arrive_threshold =
        ip["arrive_threshold"].as<double>(c.ipath.arrive_threshold);
    c.ipath.close_threshold =
        ip["close_threshold"].as<double>(c.ipath.close_threshold);
    c.ipath.ind_range = ip["ind_range"].as<int>(c.ipath.ind_range);
    c.ipath.arrive_index_threshold =
        ip["arrive_index_threshold"].as<int>(c.ipath.arrive_index_threshold);
  }

  if (const auto p = y["pan"]) {
    c.pan.iter_num = p["iter_num"].as<int>(c.pan.iter_num);
    c.pan.dune_max_num = p["dune_max_num"].as<int>(c.pan.dune_max_num);
    c.pan.iter_threshold = p["iter_threshold"].as<double>(c.pan.iter_threshold);
    c.adjust.max_num = p["nrmp_max_num"].as<int>(c.adjust.max_num);
    c.dune_checkpoint = p["dune_checkpoint"].as<std::string>("");
  }

  if (const auto a = y["adjust"]) {
    if (a["q_s"]) {
      if (a["q_s"].IsSequence()) {
        const auto v = a["q_s"].as<std::vector<double>>();
        c.adjust.q_s = Vec3(v.at(0), v.at(1), v.at(2));
      } else {
        c.adjust.q_s = Vec3::Constant(a["q_s"].as<double>());
      }
    }
    c.adjust.p_u = a["p_u"].as<double>(c.adjust.p_u);
    c.adjust.eta = a["eta"].as<double>(c.adjust.eta);
    c.adjust.d_max = a["d_max"].as<double>(c.adjust.d_max);
    c.adjust.d_min = a["d_min"].as<double>(c.adjust.d_min);
    c.adjust.ro_obs = a["ro_obs"].as<double>(c.adjust.ro_obs);
    c.adjust.bk = a["bk"].as<double>(c.adjust.bk);
  }

  if (!checkpoint_override.empty()) c.dune_checkpoint = checkpoint_override;
  if (c.dune_checkpoint.empty())
    throw std::runtime_error("neupan_cpp: dune_checkpoint (.bin) is required");

  return NeuPANPlanner(c);
}

Vec2 NeuPANPlanner::forward(const Vec3& state, const Mat2X& points,
                            Info& info, const std::optional<Command>& command) {
  info = Info{};

  if (ipath_.checkArrive(state)) {
    info.arrive = true;
    return Vec2::Zero();
  }

  const InitialPath::NomRef nom =
      ipath_.generateNomRefState(state, cur_vel_array_, ref_speed_);

  std::optional<NRMP::Box> first;
  if (command) first = firstInputBox(*command, nom.nom_u(1, 0));
  const PAN::Output out = pan_.forward(nom.nom_s, nom.nom_u, nom.ref_s,
                                       nom.ref_us, points, first);

  cur_vel_array_ = out.opt_u;

  info.solved = out.solved;
  info.solver_status = out.solver_status;
  info.min_distance = out.min_distance;
  info.opt_s = out.opt_s;
  info.opt_u = out.opt_u;
  info.ref_s = nom.ref_s;
  info.dune_points = out.dune_points;
  info.nrmp_points = out.nrmp_points;

  if (out.min_distance < cfg_.collision_threshold) {
    info.stop = true;
    return Vec2::Zero();
  }

  // An unsolved cycle returns the nominal, which the box does not bound (the
  // omni nominal can be seeded at ref_speed).
  const Vec2 u = out.opt_u.col(0)
                     .cwiseMax(-robot_.max_speed)
                     .cwiseMin(robot_.max_speed);
  if (robot_.kinematics == Kinematics::Omni)
    return Vec2(u(0) * std::cos(u(1)), u(0) * std::sin(u(1)));
  return u;
}

NRMP::Box NeuPANPlanner::firstInputBox(const Command& command,
                                       double nom_dir) const {
  const Vec2 b = robot_.acce_bound *
                 std::clamp(command.elapsed / robot_.dt, 0.0, 1.0);
  const Vec2& a = command.action;
  if (robot_.kinematics != Kinematics::Omni)
    return {(a - b).cwiseMax(-robot_.max_speed),
            (a + b).cwiseMin(robot_.max_speed)};

  // The omni input is (speed, direction): bound it round the disk of
  // velocities within b(0) of the action, with the direction taken on the
  // nominal's side of the wrap. Slower than b(0), any direction will do.
  const double speed = a.norm();
  if (speed <= b(0))
    return {Vec2(-b(0), -robot_.max_speed(1)), Vec2(b(0), robot_.max_speed(1))};
  const double dir = nom_dir + wrapToPi(std::atan2(a.y(), a.x()) - nom_dir);
  const double half = std::asin(b(0) / speed);
  return {Vec2(speed - b(0), dir - half),
          Vec2(std::min(speed + b(0), robot_.max_speed(0)), dir + half)};
}

void NeuPANPlanner::reset() {
  ipath_.reset();
  pan_.reset();
  cur_vel_array_.setZero();
}

}  // namespace neupan
