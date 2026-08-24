/* Python bindings for the standalone libneupan planner. */

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "neupan/neupan_planner.hpp"

namespace py = pybind11;

namespace {

class PyPlanner {
 public:
  PyPlanner(const std::string& config_file, const std::string& checkpoint)
      : planner_(neupan::NeuPANPlanner::fromYaml(config_file, checkpoint)) {}

  py::tuple forward(const neupan::Vec3& state, const neupan::Mat2X& points) {
    neupan::NeuPANPlanner::Info info;
    neupan::Vec2 action;
    {
      py::gil_scoped_release release;
      action = planner_.forward(state, points, info);
    }

    py::dict result;
    result["arrive"] = info.arrive;
    result["stop"] = info.stop;
    result["min_distance"] = info.min_distance;
    result["opt_s"] = std::move(info.opt_s);
    result["opt_u"] = std::move(info.opt_u);
    result["ref_s"] = std::move(info.ref_s);
    result["dune_points"] = std::move(info.dune_points);
    result["nrmp_points"] = std::move(info.nrmp_points);
    return py::make_tuple(std::move(action), std::move(result));
  }

  void setInitialPath(const Eigen::Ref<const Eigen::MatrixXd>& path) {
    if (path.cols() != 3 && path.cols() != 4)
      throw std::invalid_argument("initial path must have shape (N,3) or (N,4)");
    if (path.rows() < 2)
      throw std::invalid_argument("initial path needs at least two points");

    std::vector<neupan::InitialPath::PathPoint> converted;
    converted.reserve(static_cast<size_t>(path.rows()));
    for (Eigen::Index i = 0; i < path.rows(); ++i) {
      const double gear = path.cols() == 4 ? path(i, 3) : 1.0;
      converted.emplace_back(path(i, 0), path(i, 1), path(i, 2), gear);
    }
    planner_.setInitialPath(std::move(converted));
  }

  std::string kinematics() const {
    switch (planner_.robot().kinematics) {
      case neupan::Kinematics::Diff:
        return "diff";
      case neupan::Kinematics::Acker:
        return "acker";
    }
    throw std::logic_error("unknown kinematics");
  }

  neupan::NeuPANPlanner& planner() { return planner_; }
  const neupan::NeuPANPlanner& planner() const { return planner_; }

 private:
  neupan::NeuPANPlanner planner_;
};

}  // namespace

PYBIND11_MODULE(neupan_cpp_py, m) {
  m.doc() = "Python binding for the optimized libneupan planner";

  py::class_<PyPlanner>(m, "Planner")
      .def(py::init<const std::string&, const std::string&>(),
           py::arg("config_file"), py::arg("checkpoint"))
      .def("forward", &PyPlanner::forward, py::arg("state"), py::arg("points"))
      .def("reset", [](PyPlanner& self) { self.planner().reset(); })
      .def("set_reference_speed",
           [](PyPlanner& self, double speed) {
             self.planner().setReferenceSpeed(speed);
           })
      .def("set_initial_path", &PyPlanner::setInitialPath, py::arg("path"))
      .def("update_goal",
           [](PyPlanner& self, const neupan::Vec3& start,
              const neupan::Vec3& goal) {
             self.planner().updateInitialPathFromGoal(start, goal);
           })
      .def_property_readonly("kinematics", &PyPlanner::kinematics)
      .def_property_readonly("wheelbase", [](const PyPlanner& self) {
        return self.planner().robot().wheelbase;
      });
}
