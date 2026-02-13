#include <stdexcept>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "trochoids/trochoid_utils.h"

namespace py = pybind11;

namespace {
void copy_wind(const std::vector<double> &wind_vec, double wind[3])
{
    if (wind_vec.size() != 3)
    {
        throw std::invalid_argument("wind must contain exactly 3 elements: [wx, wy, wz]");
    }
    wind[0] = wind_vec[0];
    wind[1] = wind_vec[1];
    wind[2] = wind_vec[2];
}
}  // namespace

PYBIND11_MODULE(_trochoids, m)
{
    m.doc() = "Python bindings for the trochoids solver";

    py::enum_<trochoids::VerticalPlanningCase>(m, "VerticalPlanningCase")
        .value("NONE", trochoids::VerticalPlanningCase::NONE)
        .value("DIRECT_PROFILE", trochoids::VerticalPlanningCase::DIRECT_PROFILE)
        .value("FULL_LOOP_EXTENSION", trochoids::VerticalPlanningCase::FULL_LOOP_EXTENSION)
        .value("PARTIAL_EXTENSION", trochoids::VerticalPlanningCase::PARTIAL_EXTENSION)
        .export_values();

    py::class_<trochoids::XYZPsiState>(m, "XYZPsiState")
        .def(py::init<>())
        .def_readwrite("x", &trochoids::XYZPsiState::x)
        .def_readwrite("y", &trochoids::XYZPsiState::y)
        .def_readwrite("z", &trochoids::XYZPsiState::z)
        .def_readwrite("psi", &trochoids::XYZPsiState::psi);

    py::class_<trochoids::VerticalConstraints>(m, "VerticalConstraints")
        .def(py::init<>())
        .def_readwrite("max_climb_rate", &trochoids::VerticalConstraints::max_climb_rate)
        .def_readwrite("max_descent_rate", &trochoids::VerticalConstraints::max_descent_rate)
        .def_readwrite("enforce_flight_path_angle", &trochoids::VerticalConstraints::enforce_flight_path_angle)
        .def_readwrite("max_flight_path_angle_rad", &trochoids::VerticalConstraints::max_flight_path_angle_rad)
        .def_readwrite("allow_full_loop_extension", &trochoids::VerticalConstraints::allow_full_loop_extension)
        .def_readwrite("max_full_loops", &trochoids::VerticalConstraints::max_full_loops);

    py::class_<trochoids::VerticalPlanInfo>(m, "VerticalPlanInfo")
        .def(py::init<>())
        .def_readwrite("valid", &trochoids::VerticalPlanInfo::valid)
        .def_readwrite("vertical_feasible", &trochoids::VerticalPlanInfo::vertical_feasible)
        .def_readwrite("case_used", &trochoids::VerticalPlanInfo::case_used)
        .def_readwrite("xy_time_sec", &trochoids::VerticalPlanInfo::xy_time_sec)
        .def_readwrite("required_vertical_time_sec", &trochoids::VerticalPlanInfo::required_vertical_time_sec)
        .def_readwrite("added_extension_time_sec", &trochoids::VerticalPlanInfo::added_extension_time_sec)
        .def_readwrite("loops_added_start", &trochoids::VerticalPlanInfo::loops_added_start)
        .def_readwrite("loops_added_end", &trochoids::VerticalPlanInfo::loops_added_end)
        .def_readwrite("estimated_full_loops_needed", &trochoids::VerticalPlanInfo::estimated_full_loops_needed);

    m.def("get_trochoid_path",
          [](const trochoids::XYZPsiState &s1,
             const trochoids::XYZPsiState &s2,
             const std::vector<double> &wind_vec,
             double v,
             double max_kappa,
             double waypoint_distance) {
              double wind[3];
              copy_wind(wind_vec, wind);
              std::vector<trochoids::XYZPsiState> path;
              py::gil_scoped_release release;
              const bool valid = trochoids::get_trochoid_path(s1, s2, path, wind, v, max_kappa, waypoint_distance);
              return py::make_tuple(valid, path);
          },
          py::arg("start_state"),
          py::arg("goal_state"),
          py::arg("wind"),
          py::arg("v"),
          py::arg("max_kappa"),
          py::arg("waypoint_distance") = 0.0);

    m.def("get_trochoid_path_numerical",
          [](const trochoids::XYZPsiState &s1,
             const trochoids::XYZPsiState &s2,
             const std::vector<double> &wind_vec,
             double v,
             double max_kappa,
             bool exhaustive_solve_only,
             double waypoint_distance) {
              double wind[3];
              copy_wind(wind_vec, wind);
              std::vector<trochoids::XYZPsiState> path;
              py::gil_scoped_release release;
              const bool valid = trochoids::get_trochoid_path_numerical(
                  s1, s2, path, wind, v, max_kappa, exhaustive_solve_only, waypoint_distance);
              return py::make_tuple(valid, path);
          },
          py::arg("start_state"),
          py::arg("goal_state"),
          py::arg("wind"),
          py::arg("v"),
          py::arg("max_kappa"),
          py::arg("exhaustive_solve_only") = false,
          py::arg("waypoint_distance") = 0.0);

    m.def("get_trochoid_path_3d",
          [](const trochoids::XYZPsiState &s1,
             const trochoids::XYZPsiState &s2,
             const std::vector<double> &wind_vec,
             double v,
             double max_kappa,
             const trochoids::VerticalConstraints &constraints,
             double waypoint_distance) {
              double wind[3];
              copy_wind(wind_vec, wind);
              std::vector<trochoids::XYZPsiState> path;
              trochoids::VerticalPlanInfo info;
              py::gil_scoped_release release;
              const bool valid = trochoids::get_trochoid_path_3d(
                  s1, s2, path, wind, v, max_kappa, constraints, &info, waypoint_distance);
              return py::make_tuple(valid, path, info);
          },
          py::arg("start_state"),
          py::arg("goal_state"),
          py::arg("wind"),
          py::arg("v"),
          py::arg("max_kappa"),
          py::arg("constraints"),
          py::arg("waypoint_distance") = 0.0);
}
