// pybind11 bindings for the standalone TEB local planner library.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>

#include <teb_local_planner/teb_types.h>
#include <teb_local_planner/misc.h>
#include <teb_local_planner/distance_calculations.h>
#include <teb_local_planner/pose_se2.h>
#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/robot_footprint_model.h>
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/timed_elastic_band.h>
#include <teb_local_planner/optimal_planner.h>
#ifdef TEB_STANDALONE_WITH_HOMOTOPY
#include <teb_local_planner/homotopy_class_planner.h>
#endif

#include <boost/shared_ptr.hpp>

#include <sstream>

namespace {

struct PyObstacleContainer {
    teb_local_planner::ObstContainer storage;
};

struct PyViaPointContainer {
    teb_local_planner::ViaPointContainer storage;
};

std::vector<teb_local_planner::PoseSE2> collectPoses(const teb_local_planner::TimedElasticBand& teb) {
    std::vector<teb_local_planner::PoseSE2> poses;
    poses.reserve(static_cast<std::size_t>(teb.sizePoses()));
    for (int index = 0; index < teb.sizePoses(); ++index) {
        poses.push_back(teb.Pose(index));
    }
    return poses;
}

std::vector<double> collectTimeDiffs(const teb_local_planner::TimedElasticBand& teb) {
    std::vector<double> time_diffs;
    time_diffs.reserve(static_cast<std::size_t>(teb.sizeTimeDiffs()));
    for (int index = 0; index < teb.sizeTimeDiffs(); ++index) {
        time_diffs.push_back(teb.TimeDiff(index));
    }
    return time_diffs;
}

}  // namespace

// Declare boost::shared_ptr as a holder type for pybind11
PYBIND11_DECLARE_HOLDER_TYPE(T, boost::shared_ptr<T>)

namespace py = pybind11;
using namespace teb_local_planner;

// Trampoline class for Obstacle (pure virtual base)
class PyObstacle : public Obstacle {
public:
    using Obstacle::Obstacle;

    const Eigen::Vector2d& getCentroid() const override {
        PYBIND11_OVERRIDE_PURE(const Eigen::Vector2d&, Obstacle, getCentroid);
    }
    std::complex<double> getCentroidCplx() const override {
        PYBIND11_OVERRIDE_PURE(std::complex<double>, Obstacle, getCentroidCplx);
    }
    bool checkCollision(const Eigen::Vector2d& pos, double min_dist) const override {
        PYBIND11_OVERRIDE_PURE(bool, Obstacle, checkCollision, pos, min_dist);
    }
    bool checkLineIntersection(const Eigen::Vector2d& ls, const Eigen::Vector2d& le, double min_dist) const override {
        PYBIND11_OVERRIDE_PURE(bool, Obstacle, checkLineIntersection, ls, le, min_dist);
    }
    double getMinimumDistance(const Eigen::Vector2d& pos) const override {
        PYBIND11_OVERRIDE_PURE(double, Obstacle, getMinimumDistance, pos);
    }
    double getMinimumDistance(const Eigen::Vector2d& ls, const Eigen::Vector2d& le) const override {
        PYBIND11_OVERRIDE_PURE(double, Obstacle, getMinimumDistance, ls, le);
    }
    double getMinimumDistance(const Point2dContainer& poly) const override {
        PYBIND11_OVERRIDE_PURE(double, Obstacle, getMinimumDistance, poly);
    }
    Eigen::Vector2d getClosestPoint(const Eigen::Vector2d& pos) const override {
        PYBIND11_OVERRIDE_PURE(Eigen::Vector2d, Obstacle, getClosestPoint, pos);
    }
    double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& pos, double t) const override {
        PYBIND11_OVERRIDE_PURE(double, Obstacle, getMinimumSpatioTemporalDistance, pos, t);
    }
    double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& ls, const Eigen::Vector2d& le, double t) const override {
        PYBIND11_OVERRIDE_PURE(double, Obstacle, getMinimumSpatioTemporalDistance, ls, le, t);
    }
    double getMinimumSpatioTemporalDistance(const Point2dContainer& poly, double t) const override {
        PYBIND11_OVERRIDE_PURE(double, Obstacle, getMinimumSpatioTemporalDistance, poly, t);
    }
};

// Trampoline class for BaseRobotFootprintModel (pure virtual base)
class PyBaseRobotFootprintModel : public BaseRobotFootprintModel {
public:
    using BaseRobotFootprintModel::BaseRobotFootprintModel;

    double calculateDistance(const PoseSE2& pose, const Obstacle* obs) const override {
        PYBIND11_OVERRIDE_PURE(double, BaseRobotFootprintModel, calculateDistance, pose, obs);
    }
    double estimateSpatioTemporalDistance(const PoseSE2& pose, const Obstacle* obs, double t) const override {
        PYBIND11_OVERRIDE_PURE(double, BaseRobotFootprintModel, estimateSpatioTemporalDistance, pose, obs, t);
    }
    double getInscribedRadius() override {
        PYBIND11_OVERRIDE_PURE(double, BaseRobotFootprintModel, getInscribedRadius);
    }
};

PYBIND11_MODULE(pyteb, m) {
    m.doc() = "Python bindings for the TEB local planner standalone library";

    // -----------------------------------------------------------------------
    // RotType enum
    // -----------------------------------------------------------------------
    py::enum_<RotType>(m, "RotType")
        .value("left", RotType::left)
        .value("none", RotType::none)
        .value("right", RotType::right)
        .export_values();

    // -----------------------------------------------------------------------
    // Twist
    // -----------------------------------------------------------------------
    auto twist_cls = py::class_<Twist>(m, "Twist");

    py::class_<Twist::Linear>(twist_cls, "Linear")
        .def(py::init<>())
        .def_readwrite("x", &Twist::Linear::x)
        .def_readwrite("y", &Twist::Linear::y)
        .def_readwrite("z", &Twist::Linear::z);

    py::class_<Twist::Angular>(twist_cls, "Angular")
        .def(py::init<>())
        .def_readwrite("x", &Twist::Angular::x)
        .def_readwrite("y", &Twist::Angular::y)
        .def_readwrite("z", &Twist::Angular::z);

    twist_cls
        .def(py::init<>())
        .def(py::init<double, double, double>(),
             py::arg("lx"), py::arg("ly"), py::arg("az"))
        .def_readwrite("linear_x", &Twist::linear_x)
        .def_readwrite("linear_y", &Twist::linear_y)
        .def_readwrite("angular_z", &Twist::angular_z)
        .def_readwrite("linear", &Twist::linear)
        .def_readwrite("angular", &Twist::angular)
        .def("__repr__", [](const Twist& t) {
            std::ostringstream os;
            os << "Twist(linear_x=" << t.linear_x
               << ", linear_y=" << t.linear_y
               << ", angular_z=" << t.angular_z << ")";
            return os.str();
        });

    // -----------------------------------------------------------------------
    // PoseSE2
    // -----------------------------------------------------------------------
    py::class_<PoseSE2>(m, "PoseSE2")
        .def(py::init<>())
        .def(py::init<double, double, double>(),
             py::arg("x"), py::arg("y"), py::arg("theta"))
        .def(py::init<const Eigen::Ref<const Eigen::Vector2d>&, double>(),
             py::arg("position"), py::arg("theta"))
        .def(py::init<const PoseSE2&>(), py::arg("pose"))
        .def("x", [](const PoseSE2& p) { return p.x(); })
        .def("set_x", [](PoseSE2& p, double v) { p.x() = v; }, py::arg("val"))
        .def("y", [](const PoseSE2& p) { return p.y(); })
        .def("set_y", [](PoseSE2& p, double v) { p.y() = v; }, py::arg("val"))
        .def("theta", [](const PoseSE2& p) { return p.theta(); })
        .def("set_theta", [](PoseSE2& p, double v) { p.theta() = v; }, py::arg("val"))
        .def("position", [](const PoseSE2& p) -> Eigen::Vector2d { return p.position(); })
        .def("setZero", &PoseSE2::setZero)
        .def("orientationUnitVec", &PoseSE2::orientationUnitVec)
        .def("scale", &PoseSE2::scale, py::arg("factor"))
        .def("rotateGlobal", &PoseSE2::rotateGlobal,
             py::arg("angle"), py::arg("adjust_theta") = true)
        .def("averageInPlace", &PoseSE2::averageInPlace,
             py::arg("pose1"), py::arg("pose2"))
        .def_static("average", &PoseSE2::average,
                     py::arg("pose1"), py::arg("pose2"))
        .def(py::self + py::self)
        .def(py::self - py::self)
        .def(py::self += py::self)
        .def(py::self -= py::self)
        .def(py::self * double())
        .def(double() * py::self)
        .def("__repr__", [](const PoseSE2& p) {
            std::ostringstream os;
            os << "PoseSE2(x=" << p.x() << ", y=" << p.y()
               << ", theta=" << p.theta() << ")";
            return os.str();
        });

    // -----------------------------------------------------------------------
    // Obstacle hierarchy
    // -----------------------------------------------------------------------
    py::class_<Obstacle, PyObstacle, boost::shared_ptr<Obstacle>>(m, "Obstacle")
        .def(py::init<>())
        .def("getCentroid", &Obstacle::getCentroid)
        .def("isDynamic", &Obstacle::isDynamic)
        .def("setCentroidVelocity", &Obstacle::setCentroidVelocity, py::arg("vel"))
        .def("getCentroidVelocity", &Obstacle::getCentroidVelocity)
        .def("checkCollision",
             static_cast<bool (Obstacle::*)(const Eigen::Vector2d&, double) const>(
                 &Obstacle::checkCollision),
             py::arg("position"), py::arg("min_dist"))
        .def("checkLineIntersection",
             static_cast<bool (Obstacle::*)(const Eigen::Vector2d&, const Eigen::Vector2d&, double) const>(
                 &Obstacle::checkLineIntersection),
             py::arg("line_start"), py::arg("line_end"), py::arg("min_dist") = 0)
        .def("getMinimumDistance",
             static_cast<double (Obstacle::*)(const Eigen::Vector2d&) const>(
                 &Obstacle::getMinimumDistance),
             py::arg("position"))
        .def("getMinimumDistanceLine",
             static_cast<double (Obstacle::*)(const Eigen::Vector2d&, const Eigen::Vector2d&) const>(
                 &Obstacle::getMinimumDistance),
             py::arg("line_start"), py::arg("line_end"))
        .def("getMinimumDistancePolygon",
             static_cast<double (Obstacle::*)(const Point2dContainer&) const>(
                 &Obstacle::getMinimumDistance),
             py::arg("polygon"))
        .def("getClosestPoint", &Obstacle::getClosestPoint, py::arg("position"))
        .def("getMinimumSpatioTemporalDistance",
             static_cast<double (Obstacle::*)(const Eigen::Vector2d&, double) const>(
                 &Obstacle::getMinimumSpatioTemporalDistance),
             py::arg("position"), py::arg("t"))
        .def("getMinimumSpatioTemporalDistanceLine",
             static_cast<double (Obstacle::*)(const Eigen::Vector2d&, const Eigen::Vector2d&, double) const>(
                 &Obstacle::getMinimumSpatioTemporalDistance),
             py::arg("line_start"), py::arg("line_end"), py::arg("t"))
        .def("getMinimumSpatioTemporalDistancePolygon",
             static_cast<double (Obstacle::*)(const Point2dContainer&, double) const>(
                 &Obstacle::getMinimumSpatioTemporalDistance),
             py::arg("polygon"), py::arg("t"));

    py::class_<PointObstacle, Obstacle, boost::shared_ptr<PointObstacle>>(m, "PointObstacle")
        .def(py::init<>())
        .def(py::init<const Eigen::Ref<const Eigen::Vector2d>&>(), py::arg("position"))
        .def(py::init<double, double>(), py::arg("x"), py::arg("y"))
        .def("x", [](const PointObstacle& o) { return o.x(); })
        .def("y", [](const PointObstacle& o) { return o.y(); })
        .def("position", [](const PointObstacle& o) -> Eigen::Vector2d { return o.position(); })
        .def("__repr__", [](const PointObstacle& o) {
            std::ostringstream os;
            os << "PointObstacle(x=" << o.x() << ", y=" << o.y() << ")";
            return os.str();
        });

    py::class_<CircularObstacle, Obstacle, boost::shared_ptr<CircularObstacle>>(m, "CircularObstacle")
        .def(py::init<>())
        .def(py::init<const Eigen::Ref<const Eigen::Vector2d>&, double>(),
             py::arg("position"), py::arg("radius"))
        .def(py::init<double, double, double>(),
             py::arg("x"), py::arg("y"), py::arg("radius"))
        .def("x", [](const CircularObstacle& o) { return o.x(); })
        .def("y", [](const CircularObstacle& o) { return o.y(); })
        .def("radius", [](const CircularObstacle& o) { return o.radius(); })
        .def("position", [](const CircularObstacle& o) -> Eigen::Vector2d { return o.position(); })
        .def("__repr__", [](const CircularObstacle& o) {
            std::ostringstream os;
            os << "CircularObstacle(x=" << o.x() << ", y=" << o.y()
               << ", radius=" << o.radius() << ")";
            return os.str();
        });

    py::class_<LineObstacle, Obstacle, boost::shared_ptr<LineObstacle>>(m, "LineObstacle")
        .def(py::init<>())
        .def(py::init<const Eigen::Ref<const Eigen::Vector2d>&, const Eigen::Ref<const Eigen::Vector2d>&>(),
             py::arg("line_start"), py::arg("line_end"))
        .def(py::init<double, double, double, double>(),
             py::arg("x1"), py::arg("y1"), py::arg("x2"), py::arg("y2"))
        .def("start", &LineObstacle::start)
        .def("end", &LineObstacle::end)
        .def("setStart", &LineObstacle::setStart, py::arg("start"))
        .def("setEnd", &LineObstacle::setEnd, py::arg("end"))
        .def("__repr__", [](const LineObstacle& o) {
            std::ostringstream os;
            os << "LineObstacle(start=[" << o.start().x() << ", " << o.start().y()
               << "], end=[" << o.end().x() << ", " << o.end().y() << "])";
            return os.str();
        });

    py::class_<PillObstacle, Obstacle, boost::shared_ptr<PillObstacle>>(m, "PillObstacle")
        .def(py::init<>())
        .def(py::init<const Eigen::Ref<const Eigen::Vector2d>&, const Eigen::Ref<const Eigen::Vector2d>&, double>(),
             py::arg("line_start"), py::arg("line_end"), py::arg("radius"))
        .def(py::init<double, double, double, double, double>(),
             py::arg("x1"), py::arg("y1"), py::arg("x2"), py::arg("y2"), py::arg("radius"))
        .def("start", &PillObstacle::start)
        .def("end", &PillObstacle::end)
        .def("setStart", &PillObstacle::setStart, py::arg("start"))
        .def("setEnd", &PillObstacle::setEnd, py::arg("end"))
        .def("__repr__", [](const PillObstacle& o) {
            std::ostringstream os;
            os << "PillObstacle(start=[" << o.start().x() << ", " << o.start().y()
               << "], end=[" << o.end().x() << ", " << o.end().y() << "])";
            return os.str();
        });

    py::class_<PolygonObstacle, Obstacle, boost::shared_ptr<PolygonObstacle>>(m, "PolygonObstacle")
        .def(py::init<>())
        .def(py::init<const Point2dContainer&>(), py::arg("vertices"))
        .def("pushBackVertex",
             static_cast<void (PolygonObstacle::*)(const Eigen::Ref<const Eigen::Vector2d>&)>(
                 &PolygonObstacle::pushBackVertex),
             py::arg("vertex"))
        .def("pushBackVertexXY",
             static_cast<void (PolygonObstacle::*)(double, double)>(
                 &PolygonObstacle::pushBackVertex),
             py::arg("x"), py::arg("y"))
        .def("finalizePolygon", &PolygonObstacle::finalizePolygon)
        .def("clearVertices", &PolygonObstacle::clearVertices)
        .def("noVertices", &PolygonObstacle::noVertices)
        .def("vertices", static_cast<const Point2dContainer& (PolygonObstacle::*)() const>(&PolygonObstacle::vertices),
             py::return_value_policy::reference_internal)
        .def("__repr__", [](const PolygonObstacle& o) {
            std::ostringstream os;
            os << "PolygonObstacle(vertices=" << o.noVertices() << ")";
            return os.str();
        });

    py::class_<PyObstacleContainer>(m, "ObstacleContainer")
        .def(py::init<>())
        .def("append", [](PyObstacleContainer& container, const boost::shared_ptr<Obstacle>& obstacle) {
            container.storage.push_back(obstacle);
        }, py::arg("obstacle"))
        .def("clear", [](PyObstacleContainer& container) {
            container.storage.clear();
        })
        .def("__len__", [](const PyObstacleContainer& container) {
            return container.storage.size();
        })
        .def("__getitem__", [](const PyObstacleContainer& container, std::size_t index) {
            if (index >= container.storage.size()) {
                throw py::index_error();
            }
            return container.storage.at(index);
        })
        .def("__iter__", [](PyObstacleContainer& container) {
            return py::make_iterator(container.storage.begin(), container.storage.end());
        }, py::keep_alive<0, 1>());

    py::class_<PyViaPointContainer>(m, "ViaPointContainer")
        .def(py::init<>())
        .def("append", [](PyViaPointContainer& container, const Eigen::Ref<const Eigen::Vector2d>& via_point) {
            container.storage.push_back(via_point);
        }, py::arg("via_point"))
        .def("appendXY", [](PyViaPointContainer& container, double x, double y) {
            container.storage.emplace_back(x, y);
        }, py::arg("x"), py::arg("y"))
        .def("clear", [](PyViaPointContainer& container) {
            container.storage.clear();
        })
        .def("__len__", [](const PyViaPointContainer& container) {
            return container.storage.size();
        })
        .def("__getitem__", [](const PyViaPointContainer& container, std::size_t index) {
            if (index >= container.storage.size()) {
                throw py::index_error();
            }
            return container.storage.at(index);
        })
        .def("__iter__", [](PyViaPointContainer& container) {
            return py::make_iterator(container.storage.begin(), container.storage.end());
        }, py::keep_alive<0, 1>());

    // -----------------------------------------------------------------------
    // Robot footprint models
    // -----------------------------------------------------------------------
    py::class_<BaseRobotFootprintModel, PyBaseRobotFootprintModel,
               boost::shared_ptr<BaseRobotFootprintModel>>(m, "BaseRobotFootprintModel")
        .def(py::init<>())
        .def("calculateDistance", &BaseRobotFootprintModel::calculateDistance,
             py::arg("current_pose"), py::arg("obstacle"))
        .def("estimateSpatioTemporalDistance", &BaseRobotFootprintModel::estimateSpatioTemporalDistance,
             py::arg("current_pose"), py::arg("obstacle"), py::arg("t"))
        .def("getInscribedRadius", &BaseRobotFootprintModel::getInscribedRadius);

    py::class_<PointRobotFootprint, BaseRobotFootprintModel,
               boost::shared_ptr<PointRobotFootprint>>(m, "PointRobotFootprint")
        .def(py::init<>())
        .def(py::init<double>(), py::arg("min_obstacle_dist"));

    py::class_<CircularRobotFootprint, BaseRobotFootprintModel,
               boost::shared_ptr<CircularRobotFootprint>>(m, "CircularRobotFootprint")
        .def(py::init<double>(), py::arg("radius"))
        .def("setRadius", &CircularRobotFootprint::setRadius, py::arg("radius"));

    py::class_<TwoCirclesRobotFootprint, BaseRobotFootprintModel,
               boost::shared_ptr<TwoCirclesRobotFootprint>>(m, "TwoCirclesRobotFootprint")
        .def(py::init<double, double, double, double>(),
             py::arg("front_offset"), py::arg("front_radius"),
             py::arg("rear_offset"), py::arg("rear_radius"))
        .def("setParameters", &TwoCirclesRobotFootprint::setParameters,
             py::arg("front_offset"), py::arg("front_radius"),
             py::arg("rear_offset"), py::arg("rear_radius"));

    py::class_<LineRobotFootprint, BaseRobotFootprintModel,
               boost::shared_ptr<LineRobotFootprint>>(m, "LineRobotFootprint")
        .def(py::init<const Eigen::Vector2d&, const Eigen::Vector2d&, double>(),
             py::arg("line_start"), py::arg("line_end"), py::arg("min_obstacle_dist"))
        .def("setLine", &LineRobotFootprint::setLine,
             py::arg("line_start"), py::arg("line_end"));

    py::class_<PolygonRobotFootprint, BaseRobotFootprintModel,
               boost::shared_ptr<PolygonRobotFootprint>>(m, "PolygonRobotFootprint")
        .def(py::init<const Point2dContainer&>(), py::arg("vertices"))
        .def("setVertices", &PolygonRobotFootprint::setVertices, py::arg("vertices"));

    // -----------------------------------------------------------------------
    // TebConfig and nested structs
    // -----------------------------------------------------------------------
    auto cfg_cls = py::class_<TebConfig>(m, "TebConfig");

    py::class_<TebConfig::Trajectory>(cfg_cls, "Trajectory")
        .def(py::init<>())
        .def_readwrite("teb_autosize", &TebConfig::Trajectory::teb_autosize)
        .def_readwrite("dt_ref", &TebConfig::Trajectory::dt_ref)
        .def_readwrite("dt_hysteresis", &TebConfig::Trajectory::dt_hysteresis)
        .def_readwrite("min_samples", &TebConfig::Trajectory::min_samples)
        .def_readwrite("max_samples", &TebConfig::Trajectory::max_samples)
        .def_readwrite("global_plan_overwrite_orientation", &TebConfig::Trajectory::global_plan_overwrite_orientation)
        .def_readwrite("allow_init_with_backwards_motion", &TebConfig::Trajectory::allow_init_with_backwards_motion)
        .def_readwrite("global_plan_viapoint_sep", &TebConfig::Trajectory::global_plan_viapoint_sep)
        .def_readwrite("via_points_ordered", &TebConfig::Trajectory::via_points_ordered)
        .def_readwrite("max_global_plan_lookahead_dist", &TebConfig::Trajectory::max_global_plan_lookahead_dist)
        .def_readwrite("global_plan_prune_distance", &TebConfig::Trajectory::global_plan_prune_distance)
        .def_readwrite("exact_arc_length", &TebConfig::Trajectory::exact_arc_length)
        .def_readwrite("force_reinit_new_goal_dist", &TebConfig::Trajectory::force_reinit_new_goal_dist)
        .def_readwrite("force_reinit_new_goal_angular", &TebConfig::Trajectory::force_reinit_new_goal_angular)
        .def_readwrite("feasibility_check_no_poses", &TebConfig::Trajectory::feasibility_check_no_poses)
        .def_readwrite("feasibility_check_lookahead_distance", &TebConfig::Trajectory::feasibility_check_lookahead_distance)
        .def_readwrite("publish_feedback", &TebConfig::Trajectory::publish_feedback)
        .def_readwrite("min_resolution_collision_check_angular", &TebConfig::Trajectory::min_resolution_collision_check_angular)
        .def_readwrite("control_look_ahead_poses", &TebConfig::Trajectory::control_look_ahead_poses)
        .def_readwrite("prevent_look_ahead_poses_near_goal", &TebConfig::Trajectory::prevent_look_ahead_poses_near_goal);

    py::class_<TebConfig::Robot>(cfg_cls, "Robot")
        .def(py::init<>())
        .def_readwrite("max_vel_x", &TebConfig::Robot::max_vel_x)
        .def_readwrite("max_vel_x_backwards", &TebConfig::Robot::max_vel_x_backwards)
        .def_readwrite("max_vel_y", &TebConfig::Robot::max_vel_y)
        .def_readwrite("max_vel_trans", &TebConfig::Robot::max_vel_trans)
        .def_readwrite("max_vel_theta", &TebConfig::Robot::max_vel_theta)
        .def_readwrite("acc_lim_x", &TebConfig::Robot::acc_lim_x)
        .def_readwrite("acc_lim_y", &TebConfig::Robot::acc_lim_y)
        .def_readwrite("acc_lim_theta", &TebConfig::Robot::acc_lim_theta)
        .def_readwrite("min_turning_radius", &TebConfig::Robot::min_turning_radius)
        .def_readwrite("wheelbase", &TebConfig::Robot::wheelbase)
        .def_readwrite("cmd_angle_instead_rotvel", &TebConfig::Robot::cmd_angle_instead_rotvel)
        .def_readwrite("is_footprint_dynamic", &TebConfig::Robot::is_footprint_dynamic)
        .def_readwrite("use_proportional_saturation", &TebConfig::Robot::use_proportional_saturation)
        .def_readwrite("transform_tolerance", &TebConfig::Robot::transform_tolerance);

    py::class_<TebConfig::GoalTolerance>(cfg_cls, "GoalTolerance")
        .def(py::init<>())
        .def_readwrite("yaw_goal_tolerance", &TebConfig::GoalTolerance::yaw_goal_tolerance)
        .def_readwrite("xy_goal_tolerance", &TebConfig::GoalTolerance::xy_goal_tolerance)
        .def_readwrite("free_goal_vel", &TebConfig::GoalTolerance::free_goal_vel)
        .def_readwrite("trans_stopped_vel", &TebConfig::GoalTolerance::trans_stopped_vel)
        .def_readwrite("theta_stopped_vel", &TebConfig::GoalTolerance::theta_stopped_vel)
        .def_readwrite("complete_global_plan", &TebConfig::GoalTolerance::complete_global_plan);

    py::class_<TebConfig::Obstacles>(cfg_cls, "Obstacles")
        .def(py::init<>())
        .def_readwrite("min_obstacle_dist", &TebConfig::Obstacles::min_obstacle_dist)
        .def_readwrite("inflation_dist", &TebConfig::Obstacles::inflation_dist)
        .def_readwrite("dynamic_obstacle_inflation_dist", &TebConfig::Obstacles::dynamic_obstacle_inflation_dist)
        .def_readwrite("include_dynamic_obstacles", &TebConfig::Obstacles::include_dynamic_obstacles)
        .def_readwrite("include_costmap_obstacles", &TebConfig::Obstacles::include_costmap_obstacles)
        .def_readwrite("costmap_obstacles_behind_robot_dist", &TebConfig::Obstacles::costmap_obstacles_behind_robot_dist)
        .def_readwrite("obstacle_poses_affected", &TebConfig::Obstacles::obstacle_poses_affected)
        .def_readwrite("legacy_obstacle_association", &TebConfig::Obstacles::legacy_obstacle_association)
        .def_readwrite("obstacle_association_force_inclusion_factor", &TebConfig::Obstacles::obstacle_association_force_inclusion_factor)
        .def_readwrite("obstacle_association_cutoff_factor", &TebConfig::Obstacles::obstacle_association_cutoff_factor)
        .def_readwrite("costmap_converter_plugin", &TebConfig::Obstacles::costmap_converter_plugin)
        .def_readwrite("costmap_converter_spin_thread", &TebConfig::Obstacles::costmap_converter_spin_thread)
        .def_readwrite("costmap_converter_rate", &TebConfig::Obstacles::costmap_converter_rate)
        .def_readwrite("obstacle_proximity_ratio_max_vel", &TebConfig::Obstacles::obstacle_proximity_ratio_max_vel)
        .def_readwrite("obstacle_proximity_lower_bound", &TebConfig::Obstacles::obstacle_proximity_lower_bound)
        .def_readwrite("obstacle_proximity_upper_bound", &TebConfig::Obstacles::obstacle_proximity_upper_bound);

    py::class_<TebConfig::Optimization>(cfg_cls, "Optimization")
        .def(py::init<>())
        .def_readwrite("no_inner_iterations", &TebConfig::Optimization::no_inner_iterations)
        .def_readwrite("no_outer_iterations", &TebConfig::Optimization::no_outer_iterations)
        .def_readwrite("optimization_activate", &TebConfig::Optimization::optimization_activate)
        .def_readwrite("optimization_verbose", &TebConfig::Optimization::optimization_verbose)
        .def_readwrite("penalty_epsilon", &TebConfig::Optimization::penalty_epsilon)
        .def_readwrite("weight_max_vel_x", &TebConfig::Optimization::weight_max_vel_x)
        .def_readwrite("weight_max_vel_y", &TebConfig::Optimization::weight_max_vel_y)
        .def_readwrite("weight_max_vel_theta", &TebConfig::Optimization::weight_max_vel_theta)
        .def_readwrite("weight_acc_lim_x", &TebConfig::Optimization::weight_acc_lim_x)
        .def_readwrite("weight_acc_lim_y", &TebConfig::Optimization::weight_acc_lim_y)
        .def_readwrite("weight_acc_lim_theta", &TebConfig::Optimization::weight_acc_lim_theta)
        .def_readwrite("weight_kinematics_nh", &TebConfig::Optimization::weight_kinematics_nh)
        .def_readwrite("weight_kinematics_forward_drive", &TebConfig::Optimization::weight_kinematics_forward_drive)
        .def_readwrite("weight_kinematics_turning_radius", &TebConfig::Optimization::weight_kinematics_turning_radius)
        .def_readwrite("weight_optimaltime", &TebConfig::Optimization::weight_optimaltime)
        .def_readwrite("weight_shortest_path", &TebConfig::Optimization::weight_shortest_path)
        .def_readwrite("weight_obstacle", &TebConfig::Optimization::weight_obstacle)
        .def_readwrite("weight_inflation", &TebConfig::Optimization::weight_inflation)
        .def_readwrite("weight_dynamic_obstacle", &TebConfig::Optimization::weight_dynamic_obstacle)
        .def_readwrite("weight_dynamic_obstacle_inflation", &TebConfig::Optimization::weight_dynamic_obstacle_inflation)
        .def_readwrite("weight_velocity_obstacle_ratio", &TebConfig::Optimization::weight_velocity_obstacle_ratio)
        .def_readwrite("weight_viapoint", &TebConfig::Optimization::weight_viapoint)
        .def_readwrite("weight_prefer_rotdir", &TebConfig::Optimization::weight_prefer_rotdir)
        .def_readwrite("weight_adapt_factor", &TebConfig::Optimization::weight_adapt_factor)
        .def_readwrite("obstacle_cost_exponent", &TebConfig::Optimization::obstacle_cost_exponent);

    py::class_<TebConfig::HomotopyClasses>(cfg_cls, "HomotopyClasses")
        .def(py::init<>())
        .def_readwrite("enable_homotopy_class_planning", &TebConfig::HomotopyClasses::enable_homotopy_class_planning)
        .def_readwrite("enable_multithreading", &TebConfig::HomotopyClasses::enable_multithreading)
        .def_readwrite("simple_exploration", &TebConfig::HomotopyClasses::simple_exploration)
        .def_readwrite("max_number_classes", &TebConfig::HomotopyClasses::max_number_classes)
        .def_readwrite("max_number_plans_in_current_class", &TebConfig::HomotopyClasses::max_number_plans_in_current_class)
        .def_readwrite("selection_cost_hysteresis", &TebConfig::HomotopyClasses::selection_cost_hysteresis)
        .def_readwrite("selection_prefer_initial_plan", &TebConfig::HomotopyClasses::selection_prefer_initial_plan)
        .def_readwrite("selection_obst_cost_scale", &TebConfig::HomotopyClasses::selection_obst_cost_scale)
        .def_readwrite("selection_viapoint_cost_scale", &TebConfig::HomotopyClasses::selection_viapoint_cost_scale)
        .def_readwrite("selection_alternative_time_cost", &TebConfig::HomotopyClasses::selection_alternative_time_cost)
        .def_readwrite("selection_dropping_probability", &TebConfig::HomotopyClasses::selection_dropping_probability)
        .def_readwrite("switching_blocking_period", &TebConfig::HomotopyClasses::switching_blocking_period)
        .def_readwrite("roadmap_graph_no_samples", &TebConfig::HomotopyClasses::roadmap_graph_no_samples)
        .def_readwrite("roadmap_graph_area_width", &TebConfig::HomotopyClasses::roadmap_graph_area_width)
        .def_readwrite("roadmap_graph_area_length_scale", &TebConfig::HomotopyClasses::roadmap_graph_area_length_scale)
        .def_readwrite("h_signature_prescaler", &TebConfig::HomotopyClasses::h_signature_prescaler)
        .def_readwrite("h_signature_threshold", &TebConfig::HomotopyClasses::h_signature_threshold)
        .def_readwrite("obstacle_keypoint_offset", &TebConfig::HomotopyClasses::obstacle_keypoint_offset)
        .def_readwrite("obstacle_heading_threshold", &TebConfig::HomotopyClasses::obstacle_heading_threshold)
        .def_readwrite("viapoints_all_candidates", &TebConfig::HomotopyClasses::viapoints_all_candidates)
        .def_readwrite("visualize_hc_graph", &TebConfig::HomotopyClasses::visualize_hc_graph)
        .def_readwrite("visualize_with_time_as_z_axis_scale", &TebConfig::HomotopyClasses::visualize_with_time_as_z_axis_scale)
        .def_readwrite("delete_detours_backwards", &TebConfig::HomotopyClasses::delete_detours_backwards)
        .def_readwrite("detours_orientation_tolerance", &TebConfig::HomotopyClasses::detours_orientation_tolerance)
        .def_readwrite("length_start_orientation_vector", &TebConfig::HomotopyClasses::length_start_orientation_vector)
        .def_readwrite("max_ratio_detours_duration_best_duration", &TebConfig::HomotopyClasses::max_ratio_detours_duration_best_duration);

    py::class_<TebConfig::Recovery>(cfg_cls, "Recovery")
        .def(py::init<>())
        .def_readwrite("shrink_horizon_backup", &TebConfig::Recovery::shrink_horizon_backup)
        .def_readwrite("shrink_horizon_min_duration", &TebConfig::Recovery::shrink_horizon_min_duration)
        .def_readwrite("oscillation_recovery", &TebConfig::Recovery::oscillation_recovery)
        .def_readwrite("oscillation_v_eps", &TebConfig::Recovery::oscillation_v_eps)
        .def_readwrite("oscillation_omega_eps", &TebConfig::Recovery::oscillation_omega_eps)
        .def_readwrite("oscillation_recovery_min_duration", &TebConfig::Recovery::oscillation_recovery_min_duration)
        .def_readwrite("oscillation_filter_duration", &TebConfig::Recovery::oscillation_filter_duration)
        .def_readwrite("divergence_detection_enable", &TebConfig::Recovery::divergence_detection_enable)
        .def_readwrite("divergence_detection_max_chi_squared", &TebConfig::Recovery::divergence_detection_max_chi_squared);

    cfg_cls
        .def(py::init<>())
        .def_readwrite("odom_topic", &TebConfig::odom_topic)
        .def_readwrite("map_frame", &TebConfig::map_frame)
        .def_readwrite("robot_model", &TebConfig::robot_model)
        .def_readwrite("trajectory", &TebConfig::trajectory)
        .def_readwrite("robot", &TebConfig::robot)
        .def_readwrite("goal_tolerance", &TebConfig::goal_tolerance)
        .def_readwrite("obstacles", &TebConfig::obstacles)
        .def_readwrite("optim", &TebConfig::optim)
        .def_readwrite("hcp", &TebConfig::hcp)
        .def_readwrite("recovery", &TebConfig::recovery)
        .def("checkParameters", &TebConfig::checkParameters);

    // -----------------------------------------------------------------------
    // TimedElasticBand
    // -----------------------------------------------------------------------
    py::class_<TimedElasticBand>(m, "TimedElasticBand")
        .def(py::init<>())
        // Pose access
        .def("Pose", [](TimedElasticBand& teb, int idx) -> PoseSE2& { return teb.Pose(idx); },
             py::arg("index"), py::return_value_policy::reference_internal)
        .def("BackPose", [](TimedElasticBand& teb) -> PoseSE2& { return teb.BackPose(); },
             py::return_value_policy::reference_internal)
        // TimeDiff access
        .def("TimeDiff", [](const TimedElasticBand& teb, int idx) { return teb.TimeDiff(idx); },
             py::arg("index"))
        .def("BackTimeDiff", [](const TimedElasticBand& teb) { return teb.BackTimeDiff(); })
        // Add poses
        .def("addPose",
             static_cast<void (TimedElasticBand::*)(const PoseSE2&, bool)>(&TimedElasticBand::addPose),
             py::arg("pose"), py::arg("fixed") = false)
        .def("addPoseXYTheta",
             static_cast<void (TimedElasticBand::*)(double, double, double, bool)>(&TimedElasticBand::addPose),
             py::arg("x"), py::arg("y"), py::arg("theta"), py::arg("fixed") = false)
        // Add time diff
        .def("addTimeDiff", &TimedElasticBand::addTimeDiff,
             py::arg("dt"), py::arg("fixed") = false)
        // Add pose and time diff
        .def("addPoseAndTimeDiff",
             static_cast<void (TimedElasticBand::*)(const PoseSE2&, double)>(&TimedElasticBand::addPoseAndTimeDiff),
             py::arg("pose"), py::arg("dt"))
        .def("addPoseAndTimeDiffXYTheta",
             static_cast<void (TimedElasticBand::*)(double, double, double, double)>(&TimedElasticBand::addPoseAndTimeDiff),
             py::arg("x"), py::arg("y"), py::arg("theta"), py::arg("dt"))
        // Insert
        .def("insertPose",
             static_cast<void (TimedElasticBand::*)(int, const PoseSE2&)>(&TimedElasticBand::insertPose),
             py::arg("index"), py::arg("pose"))
        .def("insertPoseXYTheta",
             static_cast<void (TimedElasticBand::*)(int, double, double, double)>(&TimedElasticBand::insertPose),
             py::arg("index"), py::arg("x"), py::arg("y"), py::arg("theta"))
        .def("insertTimeDiff", &TimedElasticBand::insertTimeDiff,
             py::arg("index"), py::arg("dt"))
        // Delete
        .def("deletePose", &TimedElasticBand::deletePose, py::arg("index"))
        .def("deletePoses", &TimedElasticBand::deletePoses,
             py::arg("index"), py::arg("number"))
        .def("deleteTimeDiff", &TimedElasticBand::deleteTimeDiff, py::arg("index"))
        .def("deleteTimeDiffs", &TimedElasticBand::deleteTimeDiffs,
             py::arg("index"), py::arg("number"))
        // Init
        .def("initTrajectoryToGoal",
             static_cast<bool (TimedElasticBand::*)(const PoseSE2&, const PoseSE2&, double, double, int, bool)>(
                 &TimedElasticBand::initTrajectoryToGoal),
             py::arg("start"), py::arg("goal"),
             py::arg("diststep") = 0, py::arg("max_vel_x") = 0.5,
             py::arg("min_samples") = 3, py::arg("guess_backwards_motion") = false)
        // Modify
        .def("autoResize", &TimedElasticBand::autoResize,
             py::arg("dt_ref"), py::arg("dt_hysteresis"),
             py::arg("min_samples") = 3, py::arg("max_samples") = 1000,
             py::arg("fast_mode") = false)
        .def("setPoseVertexFixed", &TimedElasticBand::setPoseVertexFixed,
             py::arg("index"), py::arg("status"))
        .def("setTimeDiffVertexFixed", &TimedElasticBand::setTimeDiffVertexFixed,
             py::arg("index"), py::arg("status"))
        .def("clearTimedElasticBand", &TimedElasticBand::clearTimedElasticBand)
        // Query
        .def("sizePoses", &TimedElasticBand::sizePoses)
        .def("sizeTimeDiffs", &TimedElasticBand::sizeTimeDiffs)
        .def("isInit", &TimedElasticBand::isInit)
        .def("getSumOfAllTimeDiffs", &TimedElasticBand::getSumOfAllTimeDiffs)
        .def("getSumOfTimeDiffsUpToIdx", &TimedElasticBand::getSumOfTimeDiffsUpToIdx, py::arg("index"))
        .def("getAccumulatedDistance", &TimedElasticBand::getAccumulatedDistance)
        .def("findClosestTrajectoryPose",
             [](const TimedElasticBand& teb, const Eigen::Ref<const Eigen::Vector2d>& ref_point, int begin_idx) {
                 double dist = 0;
                 int idx = teb.findClosestTrajectoryPose(ref_point, &dist, begin_idx);
                 return py::make_tuple(idx, dist);
             },
             py::arg("ref_point"), py::arg("begin_idx") = 0)
        .def("isTrajectoryInsideRegion", &TimedElasticBand::isTrajectoryInsideRegion,
             py::arg("radius"), py::arg("max_dist_behind_robot") = -1.0,
             py::arg("skip_poses") = 0)
           .def("poses", &collectPoses)
           .def("timeDiffs", &collectTimeDiffs)
        .def("__repr__", [](const TimedElasticBand& teb) {
            std::ostringstream os;
            os << "TimedElasticBand(poses=" << teb.sizePoses()
               << ", timediffs=" << teb.sizeTimeDiffs() << ")";
            return os.str();
        });

    // -----------------------------------------------------------------------
    // PlannerInterface (abstract base, for type hierarchy)
    // -----------------------------------------------------------------------
    py::class_<PlannerInterface, boost::shared_ptr<PlannerInterface>>(m, "PlannerInterface")
        .def("clearPlanner", &PlannerInterface::clearPlanner)
        .def("setPreferredTurningDir", &PlannerInterface::setPreferredTurningDir, py::arg("dir"))
        .def("hasDiverged", &PlannerInterface::hasDiverged);

    // -----------------------------------------------------------------------
    // TebOptimalPlanner
    // -----------------------------------------------------------------------
    py::class_<TebOptimalPlanner, PlannerInterface, boost::shared_ptr<TebOptimalPlanner>>(m, "TebOptimalPlanner")
        .def(py::init<>())
        .def(py::init([](const TebConfig& cfg) {
            return new TebOptimalPlanner(cfg, nullptr, nullptr);
        }), py::arg("cfg"))
        .def(py::init([](const TebConfig& cfg, PyObstacleContainer& obstacles) {
            return new TebOptimalPlanner(cfg, &obstacles.storage, nullptr);
        }), py::arg("cfg"), py::arg("obstacles"), py::keep_alive<1, 3>())
        .def(py::init([](const TebConfig& cfg, PyObstacleContainer& obstacles, PyViaPointContainer& via_points) {
            return new TebOptimalPlanner(cfg, &obstacles.storage, &via_points.storage);
        }), py::arg("cfg"), py::arg("obstacles"), py::arg("via_points"),
             py::keep_alive<1, 3>(), py::keep_alive<1, 4>())
        .def("initialize", [](TebOptimalPlanner& planner, const TebConfig& cfg) {
            planner.initialize(cfg, nullptr, nullptr);
        }, py::arg("cfg"))
        .def("initialize", [](TebOptimalPlanner& planner, const TebConfig& cfg, PyObstacleContainer& obstacles) {
            planner.initialize(cfg, &obstacles.storage, nullptr);
        }, py::arg("cfg"), py::arg("obstacles"), py::keep_alive<1, 3>())
        .def("initialize", [](TebOptimalPlanner& planner, const TebConfig& cfg, PyObstacleContainer& obstacles, PyViaPointContainer& via_points) {
            planner.initialize(cfg, &obstacles.storage, &via_points.storage);
        }, py::arg("cfg"), py::arg("obstacles"), py::arg("via_points"),
             py::keep_alive<1, 3>(), py::keep_alive<1, 4>())
        .def("plan", [](TebOptimalPlanner& planner, const PoseSE2& start, const PoseSE2& goal, bool free_goal_vel) {
            return planner.plan(start, goal, nullptr, free_goal_vel);
        }, py::arg("start"), py::arg("goal"), py::arg("free_goal_vel") = false)
        .def("plan", [](TebOptimalPlanner& planner, const PoseSE2& start, const PoseSE2& goal, const Twist& start_vel, bool free_goal_vel) {
            return planner.plan(start, goal, &start_vel, free_goal_vel);
        }, py::arg("start"), py::arg("goal"), py::arg("start_vel"), py::arg("free_goal_vel") = false)
        .def("getVelocityCommand",
             [](const TebOptimalPlanner& planner, int look_ahead_poses) {
                 double vx = 0, vy = 0, omega = 0;
                 bool ok = planner.getVelocityCommand(vx, vy, omega, look_ahead_poses);
                 return py::make_tuple(ok, vx, vy, omega);
             },
             py::arg("look_ahead_poses") = 1)
        .def("optimizeTEB", &TebOptimalPlanner::optimizeTEB,
             py::arg("iterations_innerloop"), py::arg("iterations_outerloop"),
             py::arg("compute_cost_afterwards") = false,
             py::arg("obst_cost_scale") = 1.0, py::arg("viapoint_cost_scale") = 1.0,
             py::arg("alternative_time_cost") = false)
        .def("setVelocityStart", &TebOptimalPlanner::setVelocityStart, py::arg("vel_start"))
        .def("setVelocityGoal", &TebOptimalPlanner::setVelocityGoal, py::arg("vel_goal"))
        .def("setVelocityGoalFree", &TebOptimalPlanner::setVelocityGoalFree)
        .def("setObstVector", [](TebOptimalPlanner& planner, PyObstacleContainer& obstacles) {
            planner.setObstVector(&obstacles.storage);
        }, py::arg("obstacles"), py::keep_alive<1, 2>())
        .def("setViaPoints", [](TebOptimalPlanner& planner, PyViaPointContainer& via_points) {
            planner.setViaPoints(&via_points.storage);
        }, py::arg("via_points"), py::keep_alive<1, 2>())
        .def("teb", static_cast<TimedElasticBand& (TebOptimalPlanner::*)()>(&TebOptimalPlanner::teb),
             py::return_value_policy::reference_internal)
        .def("clearPlanner", &TebOptimalPlanner::clearPlanner)
        .def("isOptimized", &TebOptimalPlanner::isOptimized)
        .def("hasDiverged", &TebOptimalPlanner::hasDiverged)
        .def("setPreferredTurningDir", &TebOptimalPlanner::setPreferredTurningDir, py::arg("dir"))
        .def("computeCurrentCost",
             static_cast<void (TebOptimalPlanner::*)(double, double, bool)>(
                 &TebOptimalPlanner::computeCurrentCost),
             py::arg("obst_cost_scale") = 1.0, py::arg("viapoint_cost_scale") = 1.0,
             py::arg("alternative_time_cost") = false)
        .def("getCurrentCost", &TebOptimalPlanner::getCurrentCost);

#ifdef TEB_STANDALONE_WITH_HOMOTOPY
    // -----------------------------------------------------------------------
    // HomotopyClassPlanner
    // -----------------------------------------------------------------------
    py::class_<HomotopyClassPlanner, PlannerInterface, boost::shared_ptr<HomotopyClassPlanner>>(m, "HomotopyClassPlanner")
        .def(py::init<>())
        .def(py::init([](const TebConfig& cfg) {
            return new HomotopyClassPlanner(cfg, nullptr, nullptr);
        }), py::arg("cfg"))
        .def(py::init([](const TebConfig& cfg, PyObstacleContainer& obstacles) {
            return new HomotopyClassPlanner(cfg, &obstacles.storage, nullptr);
        }), py::arg("cfg"), py::arg("obstacles"), py::keep_alive<1, 3>())
        .def(py::init([](const TebConfig& cfg, PyObstacleContainer& obstacles, PyViaPointContainer& via_points) {
            return new HomotopyClassPlanner(cfg, &obstacles.storage, &via_points.storage);
        }), py::arg("cfg"), py::arg("obstacles"), py::arg("via_points"),
             py::keep_alive<1, 3>(), py::keep_alive<1, 4>())
        .def("initialize", [](HomotopyClassPlanner& planner, const TebConfig& cfg) {
            planner.initialize(cfg, nullptr, nullptr);
        }, py::arg("cfg"))
        .def("initialize", [](HomotopyClassPlanner& planner, const TebConfig& cfg, PyObstacleContainer& obstacles) {
            planner.initialize(cfg, &obstacles.storage, nullptr);
        }, py::arg("cfg"), py::arg("obstacles"), py::keep_alive<1, 3>())
        .def("initialize", [](HomotopyClassPlanner& planner, const TebConfig& cfg, PyObstacleContainer& obstacles, PyViaPointContainer& via_points) {
            planner.initialize(cfg, &obstacles.storage, &via_points.storage);
        }, py::arg("cfg"), py::arg("obstacles"), py::arg("via_points"),
             py::keep_alive<1, 3>(), py::keep_alive<1, 4>())
        .def("plan", [](HomotopyClassPlanner& planner, const PoseSE2& start, const PoseSE2& goal, bool free_goal_vel) {
            return planner.plan(start, goal, nullptr, free_goal_vel);
        }, py::arg("start"), py::arg("goal"), py::arg("free_goal_vel") = false)
        .def("plan", [](HomotopyClassPlanner& planner, const PoseSE2& start, const PoseSE2& goal, const Twist& start_vel, bool free_goal_vel) {
            return planner.plan(start, goal, &start_vel, free_goal_vel);
        }, py::arg("start"), py::arg("goal"), py::arg("start_vel"), py::arg("free_goal_vel") = false)
        .def("getVelocityCommand",
             [](const HomotopyClassPlanner& planner, int look_ahead_poses) {
                 double vx = 0, vy = 0, omega = 0;
                 bool ok = planner.getVelocityCommand(vx, vy, omega, look_ahead_poses);
                 return py::make_tuple(ok, vx, vy, omega);
             },
             py::arg("look_ahead_poses") = 1)
        .def("clearPlanner", &HomotopyClassPlanner::clearPlanner)
        .def("setPreferredTurningDir", &HomotopyClassPlanner::setPreferredTurningDir, py::arg("dir"))
        .def("hasDiverged", &HomotopyClassPlanner::hasDiverged)
        .def("bestTeb", &HomotopyClassPlanner::bestTeb)
        .def("getTrajectoryContainer", &HomotopyClassPlanner::getTrajectoryContainer,
             py::return_value_policy::reference_internal)
        .def("bestTebIdx", &HomotopyClassPlanner::bestTebIdx)
        .def("isInitialized", &HomotopyClassPlanner::isInitialized)
        .def("computeCurrentCost",
             static_cast<void (HomotopyClassPlanner::*)(std::vector<double>&, double, double, bool)>(
                 &HomotopyClassPlanner::computeCurrentCost),
             py::arg("cost"), py::arg("obst_cost_scale") = 1.0,
             py::arg("viapoint_cost_scale") = 1.0, py::arg("alternative_time_cost") = false);
#endif

    // -----------------------------------------------------------------------
    // Distance calculation free functions
    // -----------------------------------------------------------------------
    m.def("distance_point_to_segment_2d", &distance_point_to_segment_2d,
          py::arg("point"), py::arg("line_start"), py::arg("line_end"));

    m.def("check_line_segments_intersection_2d",
          [](const Eigen::Ref<const Eigen::Vector2d>& l1s, const Eigen::Ref<const Eigen::Vector2d>& l1e,
             const Eigen::Ref<const Eigen::Vector2d>& l2s, const Eigen::Ref<const Eigen::Vector2d>& l2e) {
              return check_line_segments_intersection_2d(l1s, l1e, l2s, l2e);
          },
          py::arg("line1_start"), py::arg("line1_end"),
          py::arg("line2_start"), py::arg("line2_end"));

    m.def("distance_segment_to_segment_2d", &distance_segment_to_segment_2d,
          py::arg("line1_start"), py::arg("line1_end"),
          py::arg("line2_start"), py::arg("line2_end"));

    m.def("distance_point_to_polygon_2d", &distance_point_to_polygon_2d,
          py::arg("point"), py::arg("vertices"));

    m.def("distance_segment_to_polygon_2d", &distance_segment_to_polygon_2d,
          py::arg("line_start"), py::arg("line_end"), py::arg("vertices"));

    m.def("distance_polygon_to_polygon_2d", &distance_polygon_to_polygon_2d,
          py::arg("vertices1"), py::arg("vertices2"));
}
