#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "aimee_nav_core/grid_map.hpp"
#include "aimee_nav_core/robot_centric_grid_map.hpp"
#include "aimee_nav_core/scan_matcher.hpp"
#include "aimee_nav_core/ekf_2d.hpp"
#include "aimee_nav_core/pose_graph.hpp"
#include "aimee_nav_core/global_planner.hpp"
#include "aimee_nav_core/dwa_local_planner.hpp"
#include "aimee_nav_core/mcl_2d.hpp"
#include "aimee_nav_core/frontier_detector.hpp"

namespace py = pybind11;
using namespace aimee_nav_core;

PYBIND11_MODULE(_core, m) {
    m.doc() = "AimeeNav C++ core algorithms";

    // RobotCentricGridMap (drop-in replacement for LocalGridMap)
    py::class_<RobotCentricGridMap>(m, "RobotCentricGridMap")
        .def(py::init<float, float, float, float>(),
             py::arg("size_m"), py::arg("resolution_m"), py::arg("inflation_m"), py::arg("decay_time_s") = 10.0f)
        .def("clear", &RobotCentricGridMap::clear)
        .def("update_from_scan", &RobotCentricGridMap::update_from_scan,
             py::arg("ranges"), py::arg("angles_deg"),
             py::arg("robot_x"), py::arg("robot_y"), py::arg("robot_theta"),
             py::arg("max_range_m") = 8.0f)
        .def("decay", &RobotCentricGridMap::decay)
        .def("grid_size", &RobotCentricGridMap::grid_size)
        .def("resolution", &RobotCentricGridMap::resolution)
        .def("size_m", &RobotCentricGridMap::size_m)
        .def("origin_x", &RobotCentricGridMap::origin_x)
        .def("origin_y", &RobotCentricGridMap::origin_y)
        .def("origin_theta", &RobotCentricGridMap::origin_theta)
        .def("grid_array", [](RobotCentricGridMap& self) {
            return self.data();  // Python gets a list; caller reshapes with numpy
        })
        .def("to_occupancy_grid_data", [](RobotCentricGridMap& self) {
            const auto& d = self.data();
            std::vector<int8_t> out(d.size());
            for (size_t i = 0; i < d.size(); ++i) {
                if (d[i] == 0) out[i] = -1;
                else if (d[i] < 100) out[i] = 0;
                else out[i] = 100;
            }
            return out;  // flat list
        });

    // GridMap
    py::class_<GridMap>(m, "GridMap")
        .def(py::init<float, float, float, float>(),
             py::arg("width_m"), py::arg("height_m"), py::arg("resolution_m"), py::arg("inflation_radius_m"))
        .def("clear", &GridMap::clear)
        .def("update_from_scan", &GridMap::update_from_scan,
             py::arg("origin_x"), py::arg("origin_y"), py::arg("origin_theta"),
             py::arg("ranges"), py::arg("angle_min"), py::arg("angle_increment"),
             py::arg("range_min"), py::arg("range_max"))
        .def("inflate_obstacles", &GridMap::inflate_obstacles)
        .def("extract_local_costmap", &GridMap::extract_local_costmap,
             py::arg("cx"), py::arg("cy"), py::arg("window_width_m"), py::arg("window_height_m"))
        .def("extract_local_grid_map", &GridMap::extract_local_grid_map,
             py::arg("cx"), py::arg("cy"), py::arg("window_width_m"), py::arg("window_height_m"))
        .def("world_to_grid", [](const GridMap& self, float wx, float wy) {
            int gx, gy;
            bool ok = self.world_to_grid(wx, wy, gx, gy);
            return py::make_tuple(ok, gx, gy);
        })
        .def("grid_to_world", [](const GridMap& self, int gx, int gy) {
            float wx, wy;
            self.grid_to_world(gx, gy, wx, wy);
            return py::make_tuple(wx, wy);
        })
        .def("cell", &GridMap::cell)
        .def("set_cell", &GridMap::set_cell)
        .def("inflated_cell", &GridMap::inflated_cell)
        .def("width_cells", &GridMap::width_cells)
        .def("height_cells", &GridMap::height_cells)
        .def("width_m", &GridMap::width_m)
        .def("height_m", &GridMap::height_m)
        .def("resolution_m", &GridMap::resolution_m)
        .def("set_origin", &GridMap::set_origin)
        .def("origin_x", &GridMap::origin_x)
        .def("origin_y", &GridMap::origin_y)
        .def("data", [](const GridMap& self) {
            return self.data();  // pybind11 converts std::vector<int8_t> to Python list
        })
        .def("inflated_data", [](const GridMap& self) {
            return self.inflated_data();  // pybind11 converts std::vector<uint8_t> to Python list
        })
        .def("set_data", &GridMap::set_data, py::arg("data"));

    // ScanMatcher
    py::class_<ScanMatcher>(m, "ScanMatcher")
        .def(py::init<const GridMap&>(), py::arg("map"))
        .def("match", &ScanMatcher::match,
             py::arg("ranges"), py::arg("angle_min"), py::arg("angle_increment"),
             py::arg("range_min"), py::arg("range_max"),
             py::arg("initial_x"), py::arg("initial_y"), py::arg("initial_theta"),
             py::arg("search_radius_m") = 0.5f, py::arg("search_angle_rad") = 0.2f);

    // EKF2D
    py::class_<EKF2D>(m, "EKF2D")
        .def(py::init<>())
        .def("reset", &EKF2D::reset, py::arg("x"), py::arg("y"), py::arg("theta"))
        .def("predict", &EKF2D::predict, py::arg("v"), py::arg("w"), py::arg("dt"))
        .def("update_imu_yaw", &EKF2D::update_imu_yaw, py::arg("yaw"), py::arg("yaw_variance"))
        .def("update_scan_pose", &EKF2D::update_scan_pose,
             py::arg("x"), py::arg("y"), py::arg("theta"),
             py::arg("pos_variance"), py::arg("yaw_variance"))
        .def("x", &EKF2D::x)
        .def("y", &EKF2D::y)
        .def("theta", &EKF2D::theta)
        .def("covariance", [](const EKF2D& self) {
            const auto& c = self.covariance();
            py::array_t<float> arr(9);
            std::memcpy(arr.mutable_data(), c.data(), 9 * sizeof(float));
            return arr;
        });

    // PoseGraph / Keyframe / Constraint
    py::class_<Keyframe>(m, "Keyframe")
        .def(py::init<>())
        .def_readwrite("x", &Keyframe::x)
        .def_readwrite("y", &Keyframe::y)
        .def_readwrite("theta", &Keyframe::theta)
        .def_readwrite("xs", &Keyframe::xs)
        .def_readwrite("ys", &Keyframe::ys);

    py::class_<Constraint>(m, "Constraint")
        .def(py::init<>())
        .def_readwrite("from", &Constraint::from)
        .def_readwrite("to", &Constraint::to)
        .def_readwrite("dx", &Constraint::dx)
        .def_readwrite("dy", &Constraint::dy)
        .def_readwrite("dtheta", &Constraint::dtheta);

    py::class_<PoseGraph>(m, "PoseGraph")
        .def(py::init<>())
        .def("add_keyframe", &PoseGraph::add_keyframe, py::arg("kf"))
        .def("find_nearby", &PoseGraph::find_nearby, py::arg("x"), py::arg("y"), py::arg("radius_m"))
        .def("add_constraint", &PoseGraph::add_constraint,
             py::arg("from"), py::arg("to"), py::arg("dx"), py::arg("dy"), py::arg("dtheta"))
        .def("optimize", &PoseGraph::optimize, py::arg("iterations") = 10)
        .def("keyframes", [](PoseGraph& self) -> std::vector<Keyframe>& {
            return self.keyframes();
        }, py::return_value_policy::reference_internal)
        .def("constraints", [](PoseGraph& self) -> const std::vector<Constraint>& {
            return self.constraints();
        }, py::return_value_policy::reference_internal);

    // GlobalPlanner
    py::class_<GlobalPlanner>(m, "GlobalPlanner")
        .def(py::init<>())
        .def("plan", &GlobalPlanner::plan,
             py::arg("map"), py::arg("start_x"), py::arg("start_y"),
             py::arg("goal_x"), py::arg("goal_y"));

    // MCL2D / Particle
    py::class_<Particle>(m, "Particle")
        .def_readwrite("x", &Particle::x)
        .def_readwrite("y", &Particle::y)
        .def_readwrite("theta", &Particle::theta)
        .def_readwrite("weight", &Particle::weight);

    py::class_<MCL2D>(m, "MCL2D")
        .def(py::init<>())
        .def("global_localization", &MCL2D::global_localization,
             py::arg("map"), py::arg("max_particles") = 2000)
        .def("set_initial_pose", &MCL2D::set_initial_pose,
             py::arg("x"), py::arg("y"), py::arg("theta"),
             py::arg("xy_variance"), py::arg("theta_variance"),
             py::arg("num_particles") = 500)
        .def("predict", &MCL2D::predict, py::arg("v"), py::arg("w"), py::arg("dt"))
        .def("update", &MCL2D::update,
             py::arg("ranges"), py::arg("angle_min"), py::arg("angle_increment"),
             py::arg("range_min"), py::arg("range_max"))
        .def("get_pose", &MCL2D::get_pose)
        .def("get_covariance", &MCL2D::get_covariance)
        .def("is_converged", &MCL2D::is_converged,
             py::arg("position_tolerance_m") = 0.3f,
             py::arg("angle_tolerance_rad") = 0.3f)
        .def("particles", &MCL2D::particles, py::return_value_policy::reference_internal)
        .def("set_motion_noise", &MCL2D::set_motion_noise,
             py::arg("alpha1"), py::arg("alpha2"), py::arg("alpha3"), py::arg("alpha4"))
        .def("set_min_max_particles", &MCL2D::set_min_max_particles,
             py::arg("min_p"), py::arg("max_p"))
        .def("set_kld_epsilon", &MCL2D::set_kld_epsilon, py::arg("eps"));

    // FrontierDetector / FrontierCluster
    py::class_<FrontierCluster>(m, "FrontierCluster")
        .def_readwrite("cx", &FrontierCluster::cx)
        .def_readwrite("cy", &FrontierCluster::cy)
        .def_readwrite("size", &FrontierCluster::size)
        .def_readwrite("min_x", &FrontierCluster::min_x)
        .def_readwrite("max_x", &FrontierCluster::max_x)
        .def_readwrite("min_y", &FrontierCluster::min_y)
        .def_readwrite("max_y", &FrontierCluster::max_y);

    py::class_<FrontierDetector>(m, "FrontierDetector")
        .def(py::init<>())
        .def("initialize", &FrontierDetector::initialize, py::arg("map"))
        .def("on_cell_changed", &FrontierDetector::on_cell_changed, py::arg("gx"), py::arg("gy"))
        .def("on_cells_changed", &FrontierDetector::on_cells_changed, py::arg("cells"))
        .def("get_clusters", &FrontierDetector::get_clusters, py::arg("min_size_cells"))
        .def("clear", &FrontierDetector::clear);

    // DWALocalPlanner / DWAConfig
    py::class_<DWAConfig>(m, "DWAConfig")
        .def(py::init<>())
        .def_readwrite("max_vel_x", &DWAConfig::max_vel_x)
        .def_readwrite("max_vel_theta", &DWAConfig::max_vel_theta)
        .def_readwrite("acc_lim_x", &DWAConfig::acc_lim_x)
        .def_readwrite("acc_lim_theta", &DWAConfig::acc_lim_theta)
        .def_readwrite("sim_time", &DWAConfig::sim_time)
        .def_readwrite("dt", &DWAConfig::dt)
        .def_readwrite("vx_samples", &DWAConfig::vx_samples)
        .def_readwrite("vtheta_samples", &DWAConfig::vtheta_samples)
        .def_readwrite("goal_distance_tolerance", &DWAConfig::goal_distance_tolerance)
        .def_readwrite("obstacle_distance_tolerance", &DWAConfig::obstacle_distance_tolerance)
        .def_readwrite("heading_weight", &DWAConfig::heading_weight)
        .def_readwrite("obstacle_weight", &DWAConfig::obstacle_weight)
        .def_readwrite("velocity_weight", &DWAConfig::velocity_weight)
        .def_readwrite("path_weight", &DWAConfig::path_weight);

    py::class_<DWALocalPlanner>(m, "DWALocalPlanner")
        .def(py::init<const DWAConfig&>(), py::arg("cfg") = DWAConfig{})
        .def("compute_velocity", &DWALocalPlanner::compute_velocity,
             py::arg("map"), py::arg("current_x"), py::arg("current_y"), py::arg("current_theta"),
             py::arg("current_v"), py::arg("current_w"),
             py::arg("global_path"), py::arg("goal_x"), py::arg("goal_y"));
}
