/*
 * neupan_cpp_ros: ROS2 wrapper for libneupan, the C++ port of NeuPAN.
 *
 * Ported from neupan_ros (https://github.com/hanruihua/neupan_ros),
 * Copyright (c) 2025 Ruihua Han <hanrh@connect.hku.hk>.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version. See <https://www.gnu.org/licenses/>.
 */

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "neupan/neupan_planner.hpp"

using namespace std::chrono_literals;

namespace {

double quatToYaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.z * q.z + q.y * q.y));
}

geometry_msgs::msg::Quaternion yawToQuat(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

}  // namespace

class NeuPANNode : public rclcpp::Node {
 public:
  NeuPANNode() : Node("neupan_node") {
    const auto config_file = declare_parameter<std::string>("config_file", "");
    const auto dune_checkpoint =
        declare_parameter<std::string>("dune_checkpoint", "");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "laser_link");
    marker_size_ = declare_parameter<double>("marker_size", 0.05);
    marker_z_ = declare_parameter<double>("marker_z", 1.0);
    scan_angle_min_ = declare_parameter<double>("scan_angle_min", -3.14);
    scan_angle_max_ = declare_parameter<double>("scan_angle_max", 3.14);
    scan_range_min_ = declare_parameter<double>("scan_range_min", 0.0);
    scan_range_max_ = declare_parameter<double>("scan_range_max", 5.0);
    scan_downsample_ = declare_parameter<int>("scan_downsample", 1);
    flip_angle_ = declare_parameter<bool>("flip_angle", false);
    include_initial_path_direction_ =
        declare_parameter<bool>("include_initial_path_direction", false);
    scan_timeout_ = declare_parameter<double>("scan_timeout", 0.5);
    solver_fail_grace_ = declare_parameter<int>("solver_fail_grace", 5);
    lookahead_ = declare_parameter<double>("lookahead", 0.0);
    const double rate = declare_parameter<double>("control_rate", 50.0);

    if (scan_downsample_ < 1)
      throw std::runtime_error("scan_downsample must be >= 1");

    if (config_file.empty())
      throw std::runtime_error("parameter 'config_file' is required");
    if (dune_checkpoint.empty())
      throw std::runtime_error(
          "parameter 'dune_checkpoint' (.bin model) is required");

    planner_ = std::make_unique<neupan::NeuPANPlanner>(
        neupan::NeuPANPlanner::fromYaml(config_file, dune_checkpoint));

    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/neupan_cmd_vel", 10);
    plan_pub_ = create_publisher<nav_msgs::msg::Path>("/neupan_plan", 10);
    ref_state_pub_ =
        create_publisher<nav_msgs::msg::Path>("/neupan_ref_state", 10);
    ref_path_pub_ =
        create_publisher<nav_msgs::msg::Path>("/neupan_initial_path", 10);
    dune_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/dune_point_markers", 10);
    nrmp_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/nrmp_point_markers", 10);
    robot_marker_pub_ =
        create_publisher<visualization_msgs::msg::Marker>("/robot_marker", 10);
    // A decision layer may drive its chassis output off this, so publish every cycle.
    arrive_pub_ = create_publisher<std_msgs::msg::Bool>("/neupan_arrive", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
          scanCallback(*msg);
        });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/points", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          cloudCallback(*msg);
        });
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/initial_path", 10, [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
          pathCallback(*msg);
        });
    pause_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/neupan_pause", rclcpp::QoS(1).transient_local(),
        [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
          paused_ = msg->data;
        });
    waypoints_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/neupan_waypoints", 10,
        [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
          waypointsCallback(*msg);
        });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/neupan_goal", 10,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
          goalCallback(*msg);
        });

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate), [this] { run(); });

    RCLCPP_INFO(get_logger(), "neupan_cpp node ready (config: %s)",
                config_file.c_str());
  }

 private:
  std::optional<neupan::Vec3> lookupPose(const std::string& target,
                                         const std::string& source,
                                         rclcpp::Time* stamp = nullptr) {
    try {
      const auto tfs = tf_buffer_->lookupTransform(target, source,
                                                   tf2::TimePointZero);
      if (stamp) *stamp = tfs.header.stamp;
      return neupan::Vec3(tfs.transform.translation.x,
                          tfs.transform.translation.y,
                          quatToYaw(tfs.transform.rotation));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "waiting for tf %s -> %s: %s", target.c_str(),
                           source.c_str(), ex.what());
      return std::nullopt;
    }
  }

  // Publishing nothing is not a stop: consumers hold the last command.
  void halt(const char* why) {
    vel_pub_->publish(geometry_msgs::msg::Twist());
    plan_pub_->publish(matToPath(neupan::Mat3X(3, 0)));
    publishArrive(false);
    command_ = {};
    command_time_ = get_clock()->now();
    if (why) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s", why);
  }

  void run() {
    const auto state = lookupPose(map_frame_, base_frame_, &state_stamp_);
    if (!state) {
      halt("no robot pose (TF map->base), holding still");
      return;
    }
    robot_state_ = *state;
    have_state_ = true;

    // Something else drives the robot, or nothing does: the next plan starts
    // from rest.
    if (paused_) {
      halt(nullptr);
      return;
    }

    if (scanIsStale()) {
      obstacle_points_.resize(2, 0);
      halt("scan is stale, obstacles unknown, holding still");
      return;
    }

    auto& ipath = planner_->ipath();
    if (ipath.hasConfiguredWaypoints() && !ipath.hasPath())
      ipath.setIpathWithState(robot_state_);

    if (!ipath.hasPath()) {
      halt("waiting for neupan initial path");
      return;
    }

    publishInitialPath();

    if (obstacle_points_.cols() == 0)
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "no obstacle points, only path tracking will be performed");

    // The plan starts from the last command, so the robot is never asked for
    // more than max_acce, and from where that command will have taken the
    // robot after lookahead, when a new one takes effect.
    const rclcpp::Time now = get_clock()->now();
    command_.elapsed = (now - command_time_).seconds();
    neupan::NeuPANPlanner::Info info;
    const neupan::Vec2 action = planner_->forward(
        predict(robot_state_, command_.action, lookahead_), obstacle_points_,
        info, command_);

    if (info.arrive)
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "arrive at the target");
    if (info.stop)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                           "neupan stop, min distance %.3f below threshold %.3f",
                           info.min_distance, planner_->collisionThreshold());

    geometry_msgs::msg::Twist twist;
    const bool moving = !info.stop && !info.arrive && !solverGaveUp(info);
    if (moving) {
      if (planner_->robot().kinematics == neupan::Kinematics::Omni) {
        // (vx, vy) in the map frame, turned into the base frame.
        const double c = std::cos(robot_state_(2));
        const double s = std::sin(robot_state_(2));
        twist.linear.x = c * action(0) + s * action(1);
        twist.linear.y = -s * action(0) + c * action(1);
      } else {
        twist.linear.x = action(0);
        twist.angular.z = action(1);
      }
    }
    vel_pub_->publish(twist);
    publishArrive(info.arrive);
    command_.action = moving ? action : neupan::Vec2::Zero();
    command_time_ = now;

    // A follower may track the plan instead of the twist, so it is empty when stopped.
    plan_pub_->publish(matToPath(moving ? info.opt_s : neupan::Mat3X(3, 0)));
    if (info.ref_s.cols() > 0) ref_state_pub_->publish(matToPath(info.ref_s));
    publishPointMarkers(info.dune_points, dune_markers_pub_, 160, 32, 240);
    publishPointMarkers(info.nrmp_points, nrmp_markers_pub_, 255, 128, 0);
    publishRobotMarker();
  }

  // A run of unsolved cycles means driving blind on a nominal with no avoidance.
  bool solverGaveUp(const neupan::NeuPANPlanner::Info& info) {
    if (info.solved) {
      consecutive_unsolved_ = 0;
      return false;
    }
    ++consecutive_unsolved_;
    if (consecutive_unsolved_ <= solver_fail_grace_) return false;

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                         "%d consecutive unsolved cycles (osqp status %d), stopping",
                         consecutive_unsolved_, info.solver_status);
    return true;
  }

  bool scanIsStale() const {
    if (!scan_seen_) return false;  // never had a scan: the no-points path warns
    return (now() - last_scan_time_).seconds() > scan_timeout_;
  }

  void publishArrive(bool arrived) {
    std_msgs::msg::Bool msg;
    msg.data = arrived;
    arrive_pub_->publish(msg);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan& msg) {
    if (!have_state_) return;

    const auto lidar_pose = lookupPose(map_frame_, lidar_frame_);
    if (!lidar_pose) return;

    last_scan_time_ = now();
    scan_seen_ = true;

    const int n = static_cast<int>(msg.ranges.size());
    std::vector<double> xs, ys;
    xs.reserve(n);
    ys.reserve(n);

    // Deriving the increment from the range divides by one beam too few.
    const double angle_inc =
        msg.angle_increment != 0.0f
            ? msg.angle_increment
            : (n > 1 ? (msg.angle_max - msg.angle_min) / (n - 1) : 0.0);
    for (int i = 0; i < n; ++i) {
      if (i % scan_downsample_ != 0) continue;
      const double r = msg.ranges[i];
      const double angle = flip_angle_ ? msg.angle_max - i * angle_inc
                                       : msg.angle_min + i * angle_inc;
      if (r >= scan_range_min_ && r <= scan_range_max_ &&
          angle > scan_angle_min_ && angle < scan_angle_max_) {
        xs.push_back(r * std::cos(angle));
        ys.push_back(r * std::sin(angle));
      }
    }

    if (xs.empty()) {
      obstacle_points_.resize(2, 0);
      return;
    }

    const double cy = std::cos((*lidar_pose)(2)), sy = std::sin((*lidar_pose)(2));
    neupan::Mat2X pts(2, xs.size());
    for (size_t i = 0; i < xs.size(); ++i) {
      pts(0, i) = cy * xs[i] - sy * ys[i] + (*lidar_pose)(0);
      pts(1, i) = sy * xs[i] + cy * ys[i] + (*lidar_pose)(1);
    }
    obstacle_points_ = std::move(pts);
  }

  // Obstacle points in any frame, e.g. a 3D lidar's already cut down to
  // obstacles; z is dropped.
  void cloudCallback(const sensor_msgs::msg::PointCloud2& msg) {
    if (!have_state_) return;

    geometry_msgs::msg::TransformStamped tfs;
    try {
      tfs = tf_buffer_->lookupTransform(map_frame_, msg.header.frame_id,
                                        tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "waiting for tf %s -> %s: %s", map_frame_.c_str(),
                           msg.header.frame_id.c_str(), ex.what());
      return;
    }
    const auto& t = tfs.transform;
    const Eigen::Isometry3d T =
        Eigen::Translation3d(t.translation.x, t.translation.y, t.translation.z) *
        Eigen::Quaterniond(t.rotation.w, t.rotation.x, t.rotation.y,
                           t.rotation.z);

    last_scan_time_ = now();
    scan_seen_ = true;

    neupan::Mat2X pts(2, msg.width * msg.height);
    Eigen::Index n = 0;
    sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), y(msg, "y"),
        z(msg, "z");
    for (; x != x.end(); ++x, ++y, ++z)
      pts.col(n++) = (T * Eigen::Vector3d(*x, *y, *z)).head<2>();
    obstacle_points_ = std::move(pts);
  }

  // Path poses -> [x, y, theta, gear]; theta from the points gradient unless
  // include_initial_path_direction is set (as upstream).
  std::vector<neupan::InitialPath::PathPoint> pathMsgToPoints(
      const nav_msgs::msg::Path& msg) const {
    std::vector<neupan::InitialPath::PathPoint> out;
    const size_t n = msg.poses.size();
    out.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      const auto& p = msg.poses[i].pose;
      double theta;
      if (include_initial_path_direction_) {
        theta = quatToYaw(p.orientation);
      } else if (i + 1 < n) {
        const auto& p2 = msg.poses[i + 1].pose;
        theta = std::atan2(p2.position.y - p.position.y,
                           p2.position.x - p.position.x);
      } else {
        theta = out.empty() ? quatToYaw(p.orientation) : out.back()(2);
      }
      out.emplace_back(p.position.x, p.position.y, theta, 1.0);
    }
    return out;
  }

  void pathCallback(const nav_msgs::msg::Path& msg) {
    if (msg.poses.size() < 2) {
      RCLCPP_WARN(get_logger(), "ignoring initial path with < 2 poses");
      return;
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "initial path update from given path (%zu poses)",
                         msg.poses.size());

    // Replacing rather than resetting: a resetting replan is a cold start.
    if (planner_->ipath().hasPath() && have_state_) {
      planner_->replaceInitialPath(pathMsgToPoints(msg), robot_state_);
    } else {
      planner_->setInitialPath(pathMsgToPoints(msg));
      planner_->reset();
    }
    publishArrive(false);
  }

  void waypointsCallback(const nav_msgs::msg::Path& msg) {
    if (!have_state_ || msg.poses.empty()) return;

    // Republished identical waypoints must not cold-reset the planner each tick.
    const auto pts = pathMsgToPoints(msg);
    if (samePoints(pts, last_waypoints_)) return;
    last_waypoints_ = pts;

    std::vector<neupan::Vec3> wps{robot_state_};
    for (const auto& pp : pts) wps.emplace_back(pp.head<3>());

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "initial path update from waypoints");
    planner_->setWaypoints(wps);
    planner_->reset();
    publishArrive(false);
  }

  static bool samePoints(
      const std::vector<neupan::InitialPath::PathPoint>& a,
      const std::vector<neupan::InitialPath::PathPoint>& b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i)
      if ((a[i] - b[i]).cwiseAbs().maxCoeff() > 1e-9) return false;
    return true;
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped& msg) {
    if (!have_state_) return;
    const neupan::Vec3 goal(msg.pose.position.x, msg.pose.position.y,
                            quatToYaw(msg.pose.orientation));
    RCLCPP_INFO(get_logger(), "set neupan goal: [%.2f, %.2f, %.2f]", goal(0),
                goal(1), goal(2));
    planner_->updateInitialPathFromGoal(robot_state_, goal);
    planner_->reset();
  }

  // The state the action moves the robot to after t seconds.
  neupan::Vec3 predict(const neupan::Vec3& s, const neupan::Vec2& action,
                       double t) const {
    if (planner_->robot().kinematics == neupan::Kinematics::Omni)
      return {s(0) + action(0) * t, s(1) + action(1) * t, s(2)};
    return {s(0) + action(0) * std::cos(s(2)) * t,
            s(1) + action(0) * std::sin(s(2)) * t, s(2) + action(1) * t};
  }

  // Stamped from the pose the plan starts at, lookahead after the pose read
  // from TF, one step_time per state.
  nav_msgs::msg::Path matToPath(const neupan::Mat3X& states) {
    const rclcpp::Time start =
        state_stamp_ + rclcpp::Duration::from_seconds(lookahead_);
    nav_msgs::msg::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = start;
    for (Eigen::Index i = 0; i < states.cols(); ++i) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = map_frame_;
      ps.header.stamp =
          start + rclcpp::Duration::from_seconds(i * planner_->robot().dt);
      ps.pose.position.x = states(0, i);
      ps.pose.position.y = states(1, i);
      ps.pose.orientation = yawToQuat(states(2, i));
      path.poses.push_back(ps);
    }
    return path;
  }

  void publishInitialPath() {
    const auto& path = planner_->ipath().initialPath();
    nav_msgs::msg::Path msg;
    msg.header.frame_id = map_frame_;
    msg.header.stamp = now();
    for (const auto& p : path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = map_frame_;
      ps.pose.position.x = p(0);
      ps.pose.position.y = p(1);
      ps.pose.orientation = yawToQuat(p(2));
      msg.poses.push_back(ps);
    }
    ref_path_pub_->publish(msg);
  }

  void publishPointMarkers(
      const neupan::Mat2X& points,
      const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr&
          pub,
      int r, int g, int b) {
    if (points.cols() == 0) return;
    visualization_msgs::msg::MarkerArray arr;
    for (Eigen::Index i = 0; i < points.cols(); ++i) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = map_frame_;
      m.header.stamp = now();
      m.id = static_cast<int>(i);
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.scale.x = m.scale.y = m.scale.z = marker_size_;
      m.color.a = 1.0;
      m.color.r = r / 255.0f;
      m.color.g = g / 255.0f;
      m.color.b = b / 255.0f;
      m.pose.position.x = points(0, i);
      m.pose.position.y = points(1, i);
      m.pose.position.z = 0.3;
      m.pose.orientation.w = 1.0;
      arr.markers.push_back(m);
    }
    pub->publish(arr);
  }

  void publishRobotMarker() {
    const auto& cfg = planner_->config();
    visualization_msgs::msg::Marker m;
    m.header.frame_id = map_frame_;
    m.header.stamp = now();
    m.id = 0;
    m.color.a = 1.0;
    m.color.g = 1.0;
    if (cfg.vertices.cols() > 0) {  // outline of the configured polygon
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.scale.x = 0.03;
      for (int i = 0; i <= cfg.vertices.cols(); ++i) {
        geometry_msgs::msg::Point p;
        p.x = cfg.vertices(0, i % cfg.vertices.cols());
        p.y = cfg.vertices(1, i % cfg.vertices.cols());
        p.z = marker_z_;
        m.points.push_back(p);
      }
    } else {
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.scale.x = cfg.length;
      m.scale.y = cfg.width;
      m.scale.z = marker_z_;
    }
    m.pose.position.x = robot_state_(0);
    m.pose.position.y = robot_state_(1);
    m.pose.orientation = yawToQuat(robot_state_(2));
    robot_marker_pub_->publish(m);
  }

  std::unique_ptr<neupan::NeuPANPlanner> planner_;

  std::string map_frame_, base_frame_, lidar_frame_;
  double marker_size_, marker_z_;
  double scan_angle_min_, scan_angle_max_, scan_range_min_, scan_range_max_;
  int scan_downsample_;
  bool flip_angle_, include_initial_path_direction_;
  double scan_timeout_;
  int solver_fail_grace_;
  double lookahead_;

  int consecutive_unsolved_ = 0;
  bool scan_seen_ = false;
  rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};
  std::vector<neupan::InitialPath::PathPoint> last_waypoints_;

  neupan::Vec3 robot_state_ = neupan::Vec3::Zero();
  rclcpp::Time state_stamp_{0, 0, RCL_ROS_TIME};
  neupan::NeuPANPlanner::Command command_;
  rclcpp::Time command_time_{0, 0, RCL_ROS_TIME};
  bool have_state_ = false;
  bool paused_ = false;
  neupan::Mat2X obstacle_points_ = neupan::Mat2X(2, 0);

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_, ref_state_pub_,
      ref_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      dune_markers_pub_, nrmp_markers_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arrive_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      robot_marker_pub_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_,
      waypoints_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NeuPANNode>());
  rclcpp::shutdown();
  return 0;
}
