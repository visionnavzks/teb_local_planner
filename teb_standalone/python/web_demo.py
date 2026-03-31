#!/usr/bin/env python3
import math
import os
from pathlib import Path
import random
import sys
import time

try:
    import numpy as np
    import plotly.graph_objects as go
    from dash import Dash, Input, Output, State, callback_context, dash_table, dcc, html, no_update
    from dash.exceptions import PreventUpdate
    from plotly.subplots import make_subplots
except ImportError as exc:
    raise SystemExit(
        "Missing Python dependencies for the web demo.\n"
        "Create a virtual environment and install them with:\n"
        "  python3 -m venv teb_standalone/.venv\n"
        "  source teb_standalone/.venv/bin/activate\n"
        "  python -m pip install -r teb_standalone/python/requirements.txt\n"
        "Then run:\n"
        "  python teb_standalone/python/web_demo.py"
    ) from exc


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_BUILD_DIR = SCRIPT_DIR.parent / "build"

if DEFAULT_BUILD_DIR.is_dir():
    sys.path.insert(0, str(DEFAULT_BUILD_DIR))

try:
    import pyteb as teb
except ImportError as exc:
    raise SystemExit(
        f"Unable to import pyteb. Build the pybind module first at {DEFAULT_BUILD_DIR}."
    ) from exc


X_LIMITS = (-4.5, 4.5)
Y_LIMITS = (-3.5, 3.5)
DEFAULT_ROBOT_RADIUS = 0.28
MIN_START_GOAL_DISTANCE = 5.0
START_GOAL_HANDLE_RADIUS = 0.18

OBSTACLE_LAYOUT = [
    (-2.4, -1.2, 0.55),
    (-1.0, 1.0, 0.6),
    (0.7, -0.2, 0.5),
    (2.0, 1.4, 0.55),
    (2.4, -1.5, 0.45),
]

DEFAULT_TEB_CONFIG = teb.TebConfig()

DEMO_DEFAULT_OVERRIDES = {
    "start_x": -3.6,
    "start_y": -2.1,
    "goal_x": 3.2,
    "goal_y": 1.8,
    "robot_radius": DEFAULT_ROBOT_RADIUS,
    "trajectory.dt_ref": 0.25,
    "trajectory.dt_hysteresis": 0.08,
    "trajectory.min_samples": 6,
    "trajectory.max_samples": 80,
    "trajectory.global_plan_overwrite_orientation": False,
    "robot.max_vel_x": 1.2,
    "robot.max_vel_x_backwards": 0.2,
    "robot.max_vel_trans": 1.2,
    "robot.max_vel_theta": 1.2,
    "robot.acc_lim_x": 1.5,
    "robot.acc_lim_theta": 2.0,
    "obstacles.min_obstacle_dist": 0.3,
    "obstacles.inflation_dist": 0.55,
    "obstacles.obstacle_poses_affected": 20,
    "optim.no_inner_iterations": 6,
    "optim.no_outer_iterations": 4,
    "optim.weight_optimaltime": 1.0,
    "optim.weight_obstacle": 60.0,
    "optim.weight_inflation": 0.2,
    "optim.weight_kinematics_nh": 1000.0,
    "optim.weight_kinematics_forward_drive": 10.0,
}


def get_nested_attr(obj, path):
    value = obj
    for part in path:
        value = getattr(value, part)
    return value


def set_nested_attr(obj, path, value):
    target = obj
    for part in path[:-1]:
        target = getattr(target, part)
    setattr(target, path[-1], value)


def infer_value_kind(value):
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, int):
        return "int"
    if isinstance(value, float):
        return "float"
    return "text"


def field_step(value_kind):
    if value_kind == "int":
        return 1
    if value_kind == "float":
        return "any"
    return None


def make_config_field(path):
    config_path = tuple(path.split("."))
    default_value = DEMO_DEFAULT_OVERRIDES.get(path, get_nested_attr(DEFAULT_TEB_CONFIG, config_path))
    value_kind = infer_value_kind(default_value)
    return {
        "id": path.replace(".", "__"),
        "config_path": config_path,
        "label": config_path[-1],
        "meta": path,
        "default": default_value,
        "value_kind": value_kind,
        "step": field_step(value_kind),
    }


def make_custom_field(field_id, label, default, value_kind, meta, step=None, min_value=None, max_value=None):
    return {
        "id": field_id,
        "config_path": None,
        "label": label,
        "meta": meta,
        "default": default,
        "value_kind": value_kind,
        "step": step if step is not None else field_step(value_kind),
        "min": min_value,
        "max": max_value,
    }


PARAMETER_TABS = [
    {
        "id": "scene",
        "label": "Scene",
        "description": "Start, goal, and demo-specific scene controls.",
        "fields": [
            make_custom_field("start_x", "start_x", DEMO_DEFAULT_OVERRIDES["start_x"], "float", "scene.start_x", step=0.1, min_value=X_LIMITS[0], max_value=X_LIMITS[1]),
            make_custom_field("start_y", "start_y", DEMO_DEFAULT_OVERRIDES["start_y"], "float", "scene.start_y", step=0.1, min_value=Y_LIMITS[0], max_value=Y_LIMITS[1]),
            make_custom_field("goal_x", "goal_x", DEMO_DEFAULT_OVERRIDES["goal_x"], "float", "scene.goal_x", step=0.1, min_value=X_LIMITS[0], max_value=X_LIMITS[1]),
            make_custom_field("goal_y", "goal_y", DEMO_DEFAULT_OVERRIDES["goal_y"], "float", "scene.goal_y", step=0.1, min_value=Y_LIMITS[0], max_value=Y_LIMITS[1]),
            make_custom_field("robot_radius", "robot_radius", DEMO_DEFAULT_OVERRIDES["robot_radius"], "float", "scene.robot_radius", step=0.01, min_value=0.05, max_value=1.0),
            make_custom_field("auto_replan", "auto_replan", True, "bool", "scene.auto_replan"),
        ],
    },
    {
        "id": "general",
        "label": "General",
        "description": "Top-level string parameters from TebConfig.",
        "fields": [
            make_config_field("odom_topic"),
            make_config_field("map_frame"),
        ],
    },
    {
        "id": "trajectory",
        "label": "Trajectory",
        "description": "Sampling, autosizing, and look-ahead trajectory controls.",
        "fields": [
            make_config_field("trajectory.teb_autosize"),
            make_config_field("trajectory.dt_ref"),
            make_config_field("trajectory.dt_hysteresis"),
            make_config_field("trajectory.min_samples"),
            make_config_field("trajectory.max_samples"),
            make_config_field("trajectory.global_plan_overwrite_orientation"),
            make_config_field("trajectory.allow_init_with_backwards_motion"),
            make_config_field("trajectory.global_plan_viapoint_sep"),
            make_config_field("trajectory.via_points_ordered"),
            make_config_field("trajectory.max_global_plan_lookahead_dist"),
            make_config_field("trajectory.global_plan_prune_distance"),
            make_config_field("trajectory.exact_arc_length"),
            make_config_field("trajectory.force_reinit_new_goal_dist"),
            make_config_field("trajectory.force_reinit_new_goal_angular"),
            make_config_field("trajectory.feasibility_check_no_poses"),
            make_config_field("trajectory.feasibility_check_lookahead_distance"),
            make_config_field("trajectory.publish_feedback"),
            make_config_field("trajectory.min_resolution_collision_check_angular"),
            make_config_field("trajectory.control_look_ahead_poses"),
            make_config_field("trajectory.prevent_look_ahead_poses_near_goal"),
        ],
    },
    {
        "id": "robot",
        "label": "Robot",
        "description": "Velocity, acceleration, and robot kinematics parameters.",
        "fields": [
            make_config_field("robot.max_vel_x"),
            make_config_field("robot.max_vel_x_backwards"),
            make_config_field("robot.max_vel_y"),
            make_config_field("robot.max_vel_trans"),
            make_config_field("robot.max_vel_theta"),
            make_config_field("robot.acc_lim_x"),
            make_config_field("robot.acc_lim_y"),
            make_config_field("robot.acc_lim_theta"),
            make_config_field("robot.min_turning_radius"),
            make_config_field("robot.wheelbase"),
            make_config_field("robot.cmd_angle_instead_rotvel"),
            make_config_field("robot.is_footprint_dynamic"),
            make_config_field("robot.use_proportional_saturation"),
            make_config_field("robot.transform_tolerance"),
        ],
    },
    {
        "id": "goal",
        "label": "Goal",
        "description": "Goal tolerances and terminal velocity behavior.",
        "fields": [
            make_config_field("goal_tolerance.yaw_goal_tolerance"),
            make_config_field("goal_tolerance.xy_goal_tolerance"),
            make_config_field("goal_tolerance.free_goal_vel"),
            make_config_field("goal_tolerance.trans_stopped_vel"),
            make_config_field("goal_tolerance.theta_stopped_vel"),
            make_config_field("goal_tolerance.complete_global_plan"),
        ],
    },
    {
        "id": "obstacles",
        "label": "Obstacles",
        "description": "Obstacle distances, association, and converter controls.",
        "fields": [
            make_config_field("obstacles.min_obstacle_dist"),
            make_config_field("obstacles.inflation_dist"),
            make_config_field("obstacles.dynamic_obstacle_inflation_dist"),
            make_config_field("obstacles.include_dynamic_obstacles"),
            make_config_field("obstacles.include_costmap_obstacles"),
            make_config_field("obstacles.costmap_obstacles_behind_robot_dist"),
            make_config_field("obstacles.obstacle_poses_affected"),
            make_config_field("obstacles.legacy_obstacle_association"),
            make_config_field("obstacles.obstacle_association_force_inclusion_factor"),
            make_config_field("obstacles.obstacle_association_cutoff_factor"),
            make_config_field("obstacles.costmap_converter_plugin"),
            make_config_field("obstacles.costmap_converter_spin_thread"),
            make_config_field("obstacles.costmap_converter_rate"),
            make_config_field("obstacles.obstacle_proximity_ratio_max_vel"),
            make_config_field("obstacles.obstacle_proximity_lower_bound"),
            make_config_field("obstacles.obstacle_proximity_upper_bound"),
        ],
    },
    {
        "id": "optimization",
        "label": "Optimization",
        "description": "Weights, penalties, and optimizer loop controls.",
        "fields": [
            make_config_field("optim.no_inner_iterations"),
            make_config_field("optim.no_outer_iterations"),
            make_config_field("optim.optimization_activate"),
            make_config_field("optim.optimization_verbose"),
            make_config_field("optim.penalty_epsilon"),
            make_config_field("optim.weight_max_vel_x"),
            make_config_field("optim.weight_max_vel_y"),
            make_config_field("optim.weight_max_vel_theta"),
            make_config_field("optim.weight_acc_lim_x"),
            make_config_field("optim.weight_acc_lim_y"),
            make_config_field("optim.weight_acc_lim_theta"),
            make_config_field("optim.weight_kinematics_nh"),
            make_config_field("optim.weight_kinematics_forward_drive"),
            make_config_field("optim.weight_kinematics_turning_radius"),
            make_config_field("optim.weight_optimaltime"),
            make_config_field("optim.weight_shortest_path"),
            make_config_field("optim.weight_obstacle"),
            make_config_field("optim.weight_inflation"),
            make_config_field("optim.weight_dynamic_obstacle"),
            make_config_field("optim.weight_dynamic_obstacle_inflation"),
            make_config_field("optim.weight_velocity_obstacle_ratio"),
            make_config_field("optim.weight_viapoint"),
            make_config_field("optim.weight_prefer_rotdir"),
            make_config_field("optim.weight_adapt_factor"),
            make_config_field("optim.obstacle_cost_exponent"),
        ],
    },
    {
        "id": "homotopy",
        "label": "Homotopy",
        "description": "Homotopy-class planner parameters exposed by the bindings.",
        "fields": [
            make_config_field("hcp.enable_homotopy_class_planning"),
            make_config_field("hcp.enable_multithreading"),
            make_config_field("hcp.simple_exploration"),
            make_config_field("hcp.max_number_classes"),
            make_config_field("hcp.max_number_plans_in_current_class"),
            make_config_field("hcp.selection_cost_hysteresis"),
            make_config_field("hcp.selection_prefer_initial_plan"),
            make_config_field("hcp.selection_obst_cost_scale"),
            make_config_field("hcp.selection_viapoint_cost_scale"),
            make_config_field("hcp.selection_alternative_time_cost"),
            make_config_field("hcp.selection_dropping_probability"),
            make_config_field("hcp.switching_blocking_period"),
            make_config_field("hcp.roadmap_graph_no_samples"),
            make_config_field("hcp.roadmap_graph_area_width"),
            make_config_field("hcp.roadmap_graph_area_length_scale"),
            make_config_field("hcp.h_signature_prescaler"),
            make_config_field("hcp.h_signature_threshold"),
            make_config_field("hcp.obstacle_keypoint_offset"),
            make_config_field("hcp.obstacle_heading_threshold"),
            make_config_field("hcp.viapoints_all_candidates"),
            make_config_field("hcp.visualize_hc_graph"),
            make_config_field("hcp.visualize_with_time_as_z_axis_scale"),
            make_config_field("hcp.delete_detours_backwards"),
            make_config_field("hcp.detours_orientation_tolerance"),
            make_config_field("hcp.length_start_orientation_vector"),
            make_config_field("hcp.max_ratio_detours_duration_best_duration"),
        ],
    },
    {
        "id": "recovery",
        "label": "Recovery",
        "description": "Recovery and divergence detection parameters.",
        "fields": [
            make_config_field("recovery.shrink_horizon_backup"),
            make_config_field("recovery.shrink_horizon_min_duration"),
            make_config_field("recovery.oscillation_recovery"),
            make_config_field("recovery.oscillation_v_eps"),
            make_config_field("recovery.oscillation_omega_eps"),
            make_config_field("recovery.oscillation_recovery_min_duration"),
            make_config_field("recovery.oscillation_filter_duration"),
            make_config_field("recovery.divergence_detection_enable"),
            make_config_field("recovery.divergence_detection_max_chi_squared"),
        ],
    },
]

ALL_PARAMETER_FIELDS = [field for tab in PARAMETER_TABS for field in tab["fields"]]
CONTROL_DEFAULTS = {field["id"]: field["default"] for field in ALL_PARAMETER_FIELDS}
CONFIGURABLE_FIELDS = [field for field in ALL_PARAMETER_FIELDS if field["config_path"] is not None]
ACCORDION_ITEM_IDS = [f"accordion-item-{tab['id']}" for tab in PARAMETER_TABS]

TABLE_COLUMNS = [
    {"name": "idx", "id": "idx"},
    {"name": "t [s]", "id": "time_s"},
    {"name": "x [m]", "id": "x"},
    {"name": "y [m]", "id": "y"},
    {"name": "theta [rad]", "id": "theta"},
    {"name": "dt [s]", "id": "dt"},
    {"name": "v [m/s]", "id": "linear_speed"},
    {"name": "omega [rad/s]", "id": "angular_speed"},
]


def distance(point_a, point_b):
    return math.hypot(point_a[0] - point_b[0], point_a[1] - point_b[1])


def default_obstacle_layout():
    return [
        {"x": obstacle_x, "y": obstacle_y, "radius": radius}
        for obstacle_x, obstacle_y, radius in OBSTACLE_LAYOUT
    ]


def normalize_obstacle_layout(obstacle_layout):
    if not obstacle_layout:
        return default_obstacle_layout()

    normalized = []
    for obstacle in obstacle_layout:
        normalized.append(
            {
                "x": float(obstacle["x"]),
                "y": float(obstacle["y"]),
                "radius": float(obstacle["radius"]),
            }
        )
    return normalized


def obstacle_shape(obstacle):
    return {
        "type": "circle",
        "xref": "x",
        "yref": "y",
        "x0": obstacle["x"] - obstacle["radius"],
        "x1": obstacle["x"] + obstacle["radius"],
        "y0": obstacle["y"] - obstacle["radius"],
        "y1": obstacle["y"] + obstacle["radius"],
        "line": {"color": "rgba(140, 28, 19, 0.9)", "width": 2},
        "fillcolor": "rgba(205, 86, 69, 0.72)",
        "editable": True,
        "layer": "above",
    }


def point_handle_shape(center_x, center_y, radius, line_color, fill_color):
    return {
        "type": "circle",
        "xref": "x",
        "yref": "y",
        "x0": center_x - radius,
        "x1": center_x + radius,
        "y0": center_y - radius,
        "y1": center_y + radius,
        "line": {"color": line_color, "width": 2.2},
        "fillcolor": fill_color,
        "editable": True,
        "layer": "above",
    }


def parse_shape_updates(relayout_data):
    if not relayout_data:
        return {}

    shape_updates = {}

    if "shapes" in relayout_data and isinstance(relayout_data["shapes"], list):
        shapes = relayout_data["shapes"]
        for index, shape in enumerate(shapes):
            x0 = shape.get("x0")
            x1 = shape.get("x1")
            y0 = shape.get("y0")
            y1 = shape.get("y1")
            if None in (x0, x1, y0, y1):
                continue
            shape_updates[index] = {"x0": x0, "x1": x1, "y0": y0, "y1": y1}

    for key, value in relayout_data.items():
        if not key.startswith("shapes["):
            continue
        shape_index_part, attribute = key.split("].")
        shape_index = int(shape_index_part[len("shapes[") :])
        shape_updates.setdefault(shape_index, {})[attribute] = value

    return shape_updates


def parse_shape_edit_state(relayout_data, current_layout, control_values):
    shape_updates = parse_shape_updates(relayout_data)
    if not shape_updates:
        return normalize_obstacle_layout(current_layout), dict(control_values), False

    updated_layout = [dict(obstacle) for obstacle in current_layout]
    updated_controls = dict(control_values)
    changed = False
    start_index = len(updated_layout)
    goal_index = start_index + 1

    for shape_index, attributes in shape_updates.items():
        if not {"x0", "x1", "y0", "y1"}.issubset(attributes):
            continue

        center_x = float((attributes["x0"] + attributes["x1"]) * 0.5)
        center_y = float((attributes["y0"] + attributes["y1"]) * 0.5)

        if shape_index < len(updated_layout):
            updated_layout[shape_index]["x"] = float((attributes["x0"] + attributes["x1"]) * 0.5)
            updated_layout[shape_index]["y"] = float((attributes["y0"] + attributes["y1"]) * 0.5)
            changed = True
        elif shape_index == start_index:
            updated_controls["start_x"] = round(center_x, 2)
            updated_controls["start_y"] = round(center_y, 2)
            changed = True
        elif shape_index == goal_index:
            updated_controls["goal_x"] = round(center_x, 2)
            updated_controls["goal_y"] = round(center_y, 2)
            changed = True

    return normalize_obstacle_layout(updated_layout), updated_controls, changed


def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def is_pose_collision_free(point, robot_radius, obstacle_layout):
    for obstacle in obstacle_layout:
        clearance = obstacle["radius"] + robot_radius + 0.2
        if distance(point, (obstacle["x"], obstacle["y"])) <= clearance:
            return False
    return True


def sample_position(robot_radius, obstacle_layout):
    x_margin = robot_radius + 0.2
    y_margin = robot_radius + 0.2
    for _ in range(1000):
        candidate = (
            random.uniform(X_LIMITS[0] + x_margin, X_LIMITS[1] - x_margin),
            random.uniform(Y_LIMITS[0] + y_margin, Y_LIMITS[1] - y_margin),
        )
        if is_pose_collision_free(candidate, robot_radius, obstacle_layout):
            return candidate
    raise RuntimeError("Failed to sample a collision-free pose.")


def sample_start_goal(robot_radius, obstacle_layout):
    for _ in range(1000):
        start = sample_position(robot_radius, obstacle_layout)
        goal = sample_position(robot_radius, obstacle_layout)
        if distance(start, goal) >= MIN_START_GOAL_DISTANCE:
            return start, goal
    raise RuntimeError("Failed to sample a valid start/goal pair.")


def make_pose(position, target):
    heading = math.atan2(target[1] - position[1], target[0] - position[0])
    return teb.PoseSE2(position[0], position[1], heading)


def make_goal_pose(position, start):
    heading = math.atan2(position[1] - start[1], position[0] - start[0])
    return teb.PoseSE2(position[0], position[1], heading)


def build_obstacles(obstacle_layout):
    obstacles = teb.ObstacleContainer()
    for obstacle in obstacle_layout:
        obstacles.append(teb.CircularObstacle(obstacle["x"], obstacle["y"], obstacle["radius"]))
    return obstacles


def build_config(control_values):
    cfg = teb.TebConfig()
    cfg.robot_model = teb.CircularRobotFootprint(control_values["robot_radius"])

    for field in CONFIGURABLE_FIELDS:
        raw_value = control_values[field["id"]]
        if field["value_kind"] == "int":
            value = int(raw_value)
        elif field["value_kind"] == "float":
            value = float(raw_value)
        elif field["value_kind"] == "bool":
            value = bool(raw_value)
        else:
            value = str(raw_value)
        set_nested_attr(cfg, field["config_path"], value)

    cfg.checkParameters()
    return cfg


def compute_plan(control_values, obstacle_layout=None):
    planning_started_at = time.perf_counter()
    obstacle_layout = normalize_obstacle_layout(obstacle_layout)
    start_xy = (control_values["start_x"], control_values["start_y"])
    goal_xy = (control_values["goal_x"], control_values["goal_y"])

    cfg = build_config(control_values)
    obstacles = build_obstacles(obstacle_layout)
    planner = teb.TebOptimalPlanner(cfg, obstacles)

    start = make_pose(start_xy, goal_xy)
    goal = make_goal_pose(goal_xy, start_xy)

    planner.clearPlanner()
    success = planner.plan(start, goal)

    teb_band = planner.teb()
    poses = teb_band.poses() if success else []
    time_diffs = list(teb_band.timeDiffs()) if success else []

    trajectory = np.array(
        [[pose.x(), pose.y()] for pose in poses],
        dtype=float,
    ) if poses else np.zeros((0, 2), dtype=float)
    headings = [pose.theta() for pose in poses]

    cumulative_time = [0.0]
    for dt in time_diffs:
        cumulative_time.append(cumulative_time[-1] + dt)

    segment_mid_times = []
    linear_speeds = []
    angular_speeds = []
    path_length = 0.0

    for index, dt in enumerate(time_diffs):
        if dt <= 1e-9:
            segment_mid_times.append(cumulative_time[index])
            linear_speeds.append(0.0)
            angular_speeds.append(0.0)
            continue

        dx = poses[index + 1].x() - poses[index].x()
        dy = poses[index + 1].y() - poses[index].y()
        distance_step = math.hypot(dx, dy)
        path_length += distance_step

        dtheta = wrap_to_pi(headings[index + 1] - headings[index])
        segment_mid_times.append(cumulative_time[index] + 0.5 * dt)
        linear_speeds.append(distance_step / dt)
        angular_speeds.append(dtheta / dt)

    min_clearance = float("inf")
    for pose in poses:
        for obstacle in obstacle_layout:
            clearance = (
                math.hypot(pose.x() - obstacle["x"], pose.y() - obstacle["y"])
                - obstacle["radius"]
                - control_values["robot_radius"]
            )
            min_clearance = min(min_clearance, clearance)

    if not math.isfinite(min_clearance):
        min_clearance = float("nan")

    cost = float("nan")
    command = None
    if success:
        planner.computeCurrentCost()
        cost = planner.getCurrentCost()
        command_ok, vx, _, omega = planner.getVelocityCommand(1)
        if command_ok:
            command = (vx, omega)

    table_data = []
    for index, pose in enumerate(poses):
        dt_value = time_diffs[index] if index < len(time_diffs) else None
        linear_value = linear_speeds[index] if index < len(linear_speeds) else None
        angular_value = angular_speeds[index] if index < len(angular_speeds) else None
        table_data.append(
            {
                "idx": index,
                "time_s": round(cumulative_time[index], 3),
                "x": round(pose.x(), 3),
                "y": round(pose.y(), 3),
                "theta": round(pose.theta(), 3),
                "dt": "-" if dt_value is None else round(dt_value, 3),
                "linear_speed": "-" if linear_value is None else round(linear_value, 3),
                "angular_speed": "-" if angular_value is None else round(angular_value, 3),
            }
        )

    planning_time_ms = (time.perf_counter() - planning_started_at) * 1000.0

    return {
        "success": success,
        "start": start,
        "goal": goal,
        "trajectory": trajectory,
        "headings": headings,
        "time_diffs": time_diffs,
        "cumulative_time": cumulative_time,
        "segment_mid_times": segment_mid_times,
        "linear_speeds": linear_speeds,
        "angular_speeds": angular_speeds,
        "path_length": path_length,
        "duration": cumulative_time[-1] if cumulative_time else 0.0,
        "min_clearance": min_clearance,
        "cost": cost,
        "planning_time_ms": planning_time_ms,
        "command": command,
        "table_data": table_data,
        "pose_count": len(poses),
        "diverged": planner.hasDiverged(),
        "obstacle_layout": obstacle_layout,
    }


def circle_trace(center_x, center_y, radius, color, name=None, dash_style=None):
    angles = np.linspace(0.0, 2.0 * math.pi, 90)
    x = center_x + radius * np.cos(angles)
    y = center_y + radius * np.sin(angles)
    return go.Scatter(
        x=x,
        y=y,
        mode="lines",
        fill="toself",
        line={"color": color, "width": 1.5, "dash": dash_style},
        fillcolor=color if dash_style is None else "rgba(0,0,0,0)",
        opacity=0.26 if dash_style is None else 1.0,
        hoverinfo="skip",
        name=name,
        showlegend=name is not None,
    )


def heading_arrow_annotation(pose, color, length=0.58, width=2.8):
    end_x = pose.x() + length * math.cos(pose.theta())
    end_y = pose.y() + length * math.sin(pose.theta())
    return {
        "x": float(end_x),
        "y": float(end_y),
        "ax": float(pose.x()),
        "ay": float(pose.y()),
        "xref": "x",
        "yref": "y",
        "axref": "x",
        "ayref": "y",
        "showarrow": True,
        "arrowhead": 3,
        "arrowsize": 1.3,
        "arrowwidth": width,
        "arrowcolor": color,
        "opacity": 0.95,
        "text": "",
    }


def rgba(color, alpha):
    color = color.lstrip("#")
    red = int(color[0:2], 16)
    green = int(color[2:4], 16)
    blue = int(color[4:6], 16)
    return f"rgba({red}, {green}, {blue}, {alpha})"


def classify_segment_directions(trajectory, headings):
    if len(trajectory) < 2:
        return []

    directions = []
    for index in range(len(trajectory) - 1):
        dx = trajectory[index + 1, 0] - trajectory[index, 0]
        dy = trajectory[index + 1, 1] - trajectory[index, 1]
        projection = dx * math.cos(headings[index]) + dy * math.sin(headings[index])
        directions.append("forward" if projection >= 0.0 else "backward")
    return directions


def build_pose_arrow_annotations(trajectory, headings, segment_directions):
    if len(trajectory) == 0:
        return []

    annotations = []

    for index, (point_x, point_y) in enumerate(trajectory):
        if segment_directions:
            direction = segment_directions[index] if index < len(segment_directions) else segment_directions[-1]
        else:
            direction = "forward"
        color = rgba("#16a34a", 0.9) if direction == "forward" else rgba("#dc2626", 0.9)
        arrow_length = 0.22
        end_x = point_x + arrow_length * math.cos(headings[index])
        end_y = point_y + arrow_length * math.sin(headings[index])
        annotations.append(
            {
                "x": float(end_x),
                "y": float(end_y),
                "ax": float(point_x),
                "ay": float(point_y),
                "xref": "x",
                "yref": "y",
                "axref": "x",
                "ayref": "y",
                "showarrow": True,
                "arrowhead": 3,
                "arrowsize": 0.9,
                "arrowwidth": 1.15,
                "arrowcolor": color,
                "opacity": 0.92,
                "text": "",
            }
        )

    return annotations


def add_trajectory_traces(fig, trajectory, headings):
    if len(trajectory) < 2:
        return []

    segment_directions = classify_segment_directions(trajectory, headings)
    forward_color = rgba("#16a34a", 0.5)
    backward_color = rgba("#dc2626", 0.5)

    for index, direction in enumerate(segment_directions):
        color = forward_color if direction == "forward" else backward_color
        trace_name = None
        if direction == "forward" and not any(item == "forward" for item in segment_directions[:index]):
            trace_name = "Forward trajectory"
        if direction == "backward" and not any(item == "backward" for item in segment_directions[:index]):
            trace_name = "Backward trajectory"

        fig.add_trace(
            go.Scatter(
                x=trajectory[index : index + 2, 0],
                y=trajectory[index : index + 2, 1],
                mode="lines",
                line={"color": color, "width": 4},
                name=trace_name,
                showlegend=trace_name is not None,
                hovertemplate=(
                    f"segment {index}<br>"
                    f"mode {direction}<br>"
                    "x %{x:.2f} m<br>y %{y:.2f} m<extra></extra>"
                ),
            ),
            row=1,
            col=1,
        )

    return build_pose_arrow_annotations(trajectory, headings, segment_directions)


def build_figure(plan_result, control_values, obstacle_layout=None):
    obstacle_layout = normalize_obstacle_layout(obstacle_layout)
    fig = make_subplots(
        rows=2,
        cols=1,
        row_heights=[0.78, 0.22],
        vertical_spacing=0.12,
        specs=[[{"type": "scatter"}], [{"secondary_y": True}]],
        subplot_titles=("Workspace and optimized trajectory", "Velocity profile"),
    )

    inflation_radius_offset = control_values["obstacles__inflation_dist"]
    clearance_radius_offset = control_values["obstacles__min_obstacle_dist"]

    fig.update_layout(
        shapes=[obstacle_shape(obstacle) for obstacle in obstacle_layout]
        + [
            point_handle_shape(
                plan_result["start"].x(),
                plan_result["start"].y(),
                START_GOAL_HANDLE_RADIUS,
                rgba("#0f766e", 0.96),
                rgba("#0f766e", 0.12),
            ),
            point_handle_shape(
                plan_result["goal"].x(),
                plan_result["goal"].y(),
                START_GOAL_HANDLE_RADIUS,
                rgba("#dd6b4d", 0.96),
                rgba("#dd6b4d", 0.12),
            ),
        ]
    )

    for index, obstacle in enumerate(obstacle_layout):
        fig.add_trace(
            circle_trace(
                obstacle["x"],
                obstacle["y"],
                obstacle["radius"],
                "rgba(205, 86, 69, 0.001)",
                name="Obstacle" if index == 0 else None,
            ),
            row=1,
            col=1,
        )
        fig.add_trace(
            circle_trace(
                obstacle["x"],
                obstacle["y"],
                obstacle["radius"] + inflation_radius_offset,
                "rgba(236, 184, 128, 0.7)",
                name="Inflation" if index == 0 else None,
                dash_style="dot",
            ),
            row=1,
            col=1,
        )
        fig.add_trace(
            circle_trace(
                obstacle["x"],
                obstacle["y"],
                obstacle["radius"] + control_values["robot_radius"] + clearance_radius_offset,
                "rgba(15, 118, 110, 0.45)",
                name="Robot clearance" if index == 0 else None,
                dash_style="dash",
            ),
            row=1,
            col=1,
        )

    trajectory = plan_result["trajectory"]
    trajectory_annotations = add_trajectory_traces(fig, trajectory, plan_result["headings"])

    if len(trajectory) > 0:
        fig.add_trace(
            go.Scatter(
                x=trajectory[:, 0],
                y=trajectory[:, 1],
                mode="markers",
                marker={
                    "size": 8,
                    "symbol": "circle-open",
                    "color": rgba("#22303c", 0.5),
                    "line": {"width": 1.6, "color": rgba("#22303c", 0.5)},
                },
                name="Trajectory points",
                hovertemplate="x %{x:.2f} m<br>y %{y:.2f} m<extra></extra>",
            ),
            row=1,
            col=1,
        )

    fig.add_trace(
        go.Scatter(
            x=[plan_result["start"].x()],
            y=[plan_result["start"].y()],
            mode="markers",
            marker={"size": 16, "color": "#0f766e", "symbol": "circle"},
            name="Start",
        ),
        row=1,
        col=1,
    )
    fig.add_trace(
        go.Scatter(
            x=[plan_result["goal"].x()],
            y=[plan_result["goal"].y()],
            mode="markers",
            marker={"size": 16, "color": "#dd6b4d", "symbol": "diamond"},
            name="Goal",
        ),
        row=1,
        col=1,
    )
    if plan_result["segment_mid_times"]:
        fig.add_trace(
            go.Scatter(
                x=plan_result["segment_mid_times"],
                y=plan_result["linear_speeds"],
                mode="lines+markers",
                line={"color": "#0f766e", "width": 3},
                marker={"size": 6},
                name="Linear speed",
            ),
            row=2,
            col=1,
            secondary_y=False,
        )
        fig.add_trace(
            go.Scatter(
                x=plan_result["segment_mid_times"],
                y=plan_result["angular_speeds"],
                mode="lines+markers",
                line={"color": "#7c3aed", "width": 3},
                marker={"size": 6},
                name="Angular speed",
            ),
            row=2,
            col=1,
            secondary_y=True,
        )

    fig.update_xaxes(range=X_LIMITS, title_text="x [m]", row=1, col=1)
    fig.update_yaxes(range=Y_LIMITS, title_text="y [m]", row=1, col=1, scaleanchor="x", scaleratio=1)
    fig.update_xaxes(title_text="time [s]", row=2, col=1)
    fig.update_yaxes(title_text="linear speed [m/s]", row=2, col=1, secondary_y=False)
    fig.update_yaxes(title_text="angular speed [rad/s]", row=2, col=1, secondary_y=True)
    fig.update_annotations(font={"size": 15, "color": "#25313c"})
    fig.update_layout(
        height=900,
        paper_bgcolor="rgba(0,0,0,0)",
        plot_bgcolor="rgba(255,255,255,0.4)",
        margin={"l": 32, "r": 32, "t": 72, "b": 32},
        legend={"orientation": "h", "yanchor": "bottom", "y": 1.02, "x": 0.0},
        font={"family": "Avenir Next, Helvetica Neue, sans-serif", "size": 13, "color": "#22303c"},
        hovermode="closest",
        newshape={"line": {"color": "rgba(140, 28, 19, 0.9)", "width": 2}},
    )
    fig.update_layout(
        annotations=list(fig.layout.annotations)
        + trajectory_annotations
        + [
            heading_arrow_annotation(plan_result["start"], rgba("#0f766e", 0.96)),
            heading_arrow_annotation(plan_result["goal"], rgba("#dd6b4d", 0.96)),
        ]
    )
    fig.update_xaxes(showgrid=True, gridcolor="rgba(34, 48, 60, 0.09)")
    fig.update_yaxes(showgrid=True, gridcolor="rgba(34, 48, 60, 0.09)")
    return fig


def format_metric(value, unit="", precision=2):
    if value is None:
        return "--"
    if isinstance(value, str):
        return value
    if not math.isfinite(value):
        return "--"
    return f"{value:.{precision}f}{unit}"


def metric_card(metric_id, label):
    return html.Div(
        [
            html.Div(label, className="metric-label"),
            html.Div("--", id=metric_id, className="metric-value"),
        ],
        className="metric-card",
    )


def render_field(field):
    field_children = [
        html.Label(field["label"], htmlFor=field["id"], className="field-label"),
        html.Div(field["meta"], className="field-meta"),
    ]

    if field["value_kind"] == "bool":
        field_children.append(
            dcc.RadioItems(
                id=field["id"],
                options=[
                    {"label": "Off", "value": False},
                    {"label": "On", "value": True},
                ],
                value=field["default"],
                className="toggle-group",
                inputClassName="toggle-radio",
                labelClassName="toggle-option",
                inline=True,
            )
        )
    elif field["value_kind"] == "text":
        field_children.append(
            dcc.Input(
                id=field["id"],
                type="text",
                value=field["default"],
                className="text-input",
                debounce=True,
            )
        )
    else:
        field_children.append(
            dcc.Input(
                id=field["id"],
                type="number",
                value=field["default"],
                min=field.get("min"),
                max=field.get("max"),
                step=field.get("step"),
                className="number-input",
                debounce=True,
            )
        )

    return html.Div(field_children, className="field-row")


def render_parameter_accordion():
    default_open_tabs = {"scene", "trajectory"}
    return html.Div(
        [
            html.Details(
                [
                    html.Summary(
                        [
                            html.Span(tab["label"], className="accordion-summary-title"),
                            html.Span(f"{len(tab['fields'])} params", className="accordion-summary-meta"),
                        ],
                        className="accordion-summary",
                    ),
                    html.Div(
                        [
                            html.P(tab["description"], className="tab-copy"),
                            html.Div(
                                [render_field(field) for field in tab["fields"]],
                                className="tab-field-list",
                            ),
                        ],
                        className="accordion-panel",
                    ),
                ],
                id=f"accordion-item-{tab['id']}",
                className="accordion-item",
                open=tab["id"] in default_open_tabs,
            )
            for tab in PARAMETER_TABS
        ],
        className="accordion-stack",
    )


def build_status_banner(text, success):
    class_name = "status-banner status-success" if success else "status-banner status-failure"
    return html.Div(text, className=class_name)


def build_layout(initial_figure):
    return html.Div(
        className="app-shell",
        children=[
            html.Div(
                className="hero-bar panel",
                children=[
                    html.Div(
                        [
                            html.P("TEB standalone", className="eyebrow"),
                            html.H1("Web planner workbench", className="page-title"),
                            html.P(
                                "Left side edits planner parameters, the center renders Plotly output, and the right side streams the numeric result.",
                                className="page-subtitle",
                            ),
                        ]
                    ),
                    html.Div(
                        [
                            html.Button("Randomize", id="randomize-button", n_clicks=0, className="secondary-button"),
                            html.Button("Replan", id="replan-button", n_clicks=0, className="primary-button"),
                        ],
                        className="hero-actions",
                    ),
                ],
            ),
            html.Div(
                className="dashboard-grid",
                children=[
                    html.Div(
                        className="panel control-panel",
                        children=[
                            html.Div(
                                [
                                    html.Div(
                                        [
                                            html.H2("Parameters", className="panel-title"),
                                            html.P(
                                                "Use the accordion to browse all exposed TebConfig categories and scene controls.",
                                                className="panel-copy",
                                            ),
                                        ]
                                    ),
                                    html.Button(
                                        "Collapse all",
                                        id="collapse-all-button",
                                        n_clicks=0,
                                        className="secondary-button compact-button",
                                    ),
                                ],
                                className="panel-header-row",
                            ),
                            render_parameter_accordion(),
                        ],
                    ),
                    html.Div(
                        className="panel plot-panel",
                        children=[
                            html.Div(
                                [
                                    html.H2("Plotly output", className="panel-title"),
                                    html.P(
                                        "Workspace, optimized trajectory, and velocity traces update after each planning run.",
                                        className="panel-copy",
                                    ),
                                ]
                            ),
                            dcc.Graph(
                                id="planner-figure",
                                figure=initial_figure,
                                className="planner-graph",
                                config={
                                    "editable": True,
                                    "edits": {"shapePosition": True},
                                    "displaylogo": False,
                                },
                            ),
                        ],
                    ),
                    html.Div(
                        className="panel data-panel",
                        children=[
                            html.Div(
                                [
                                    html.H2("Plan data", className="panel-title"),
                                    html.P(
                                        "Metrics, command output, and the full trajectory stream live on the right.",
                                        className="panel-copy",
                                    ),
                                ]
                            ),
                            html.Div(id="status-banner"),
                            html.Div(
                                className="metric-grid",
                                children=[
                                    metric_card("metric-success", "Plan status"),
                                    metric_card("metric-runtime", "Planning time"),
                                    metric_card("metric-poses", "Pose count"),
                                    metric_card("metric-duration", "Duration"),
                                    metric_card("metric-path-length", "Path length"),
                                    metric_card("metric-clearance", "Min clearance"),
                                    metric_card("metric-cost", "Current cost"),
                                    metric_card("metric-command", "Cmd preview"),
                                    metric_card("metric-diverged", "Diverged"),
                                ],
                            ),
                            html.Div(
                                [
                                    html.H3("Trajectory samples", className="section-title"),
                                    dash_table.DataTable(
                                        id="trajectory-table",
                                        columns=TABLE_COLUMNS,
                                        data=[],
                                        page_size=14,
                                        sort_action="native",
                                        style_as_list_view=True,
                                        style_table={"overflowX": "auto"},
                                        style_header={
                                            "backgroundColor": "rgba(15, 118, 110, 0.10)",
                                            "border": "none",
                                            "color": "#1f2d3a",
                                            "fontWeight": 700,
                                        },
                                        style_cell={
                                            "backgroundColor": "rgba(255,255,255,0.45)",
                                            "border": "none",
                                            "color": "#25313c",
                                            "fontFamily": "IBM Plex Mono, Menlo, monospace",
                                            "fontSize": "12px",
                                            "padding": "8px 10px",
                                            "textAlign": "right",
                                        },
                                        style_data_conditional=[
                                            {
                                                "if": {"row_index": "odd"},
                                                "backgroundColor": "rgba(15, 118, 110, 0.04)",
                                            }
                                        ],
                                    ),
                                ],
                                className="table-section",
                            ),
                        ],
                    ),
                ],
            ),
            dcc.Store(id="obstacle-layout-store", data=default_obstacle_layout()),
            dcc.Store(id="scene-update-store", data={"source": "initial", "nonce": 0}),
        ],
    )


def collect_control_values(raw_values):
    values = {}
    for key, default_value in CONTROL_DEFAULTS.items():
        raw_value = raw_values.get(key)
        values[key] = default_value if raw_value is None else raw_value
    return values


def create_app():
    initial_values = collect_control_values(CONTROL_DEFAULTS)
    initial_obstacle_layout = default_obstacle_layout()
    try:
        initial_plan = compute_plan(initial_values, initial_obstacle_layout)
    except Exception:
        initial_plan = {
            "start": make_pose(
                (initial_values["start_x"], initial_values["start_y"]),
                (initial_values["goal_x"], initial_values["goal_y"]),
            ),
            "goal": make_goal_pose(
                (initial_values["goal_x"], initial_values["goal_y"]),
                (initial_values["start_x"], initial_values["start_y"]),
            ),
            "trajectory": np.zeros((0, 2), dtype=float),
            "segment_mid_times": [],
            "linear_speeds": [],
            "angular_speeds": [],
        }
    app = Dash(__name__, assets_folder=str(SCRIPT_DIR / "assets"), title="TEB Web Demo")
    app.layout = build_layout(build_figure(initial_plan, initial_values, initial_obstacle_layout))

    scene_callback_outputs = [
        Output("obstacle-layout-store", "data"),
        Output("start_x", "value"),
        Output("start_y", "value"),
        Output("goal_x", "value"),
        Output("goal_y", "value"),
        Output("scene-update-store", "data"),
    ]

    planner_callback_outputs = [
        Output("planner-figure", "figure"),
        Output("status-banner", "children"),
        Output("metric-success", "children"),
        Output("metric-runtime", "children"),
        Output("metric-poses", "children"),
        Output("metric-duration", "children"),
        Output("metric-path-length", "children"),
        Output("metric-clearance", "children"),
        Output("metric-cost", "children"),
        Output("metric-command", "children"),
        Output("metric-diverged", "children"),
        Output("trajectory-table", "data"),
    ]

    scene_callback_inputs = [
        Input("randomize-button", "n_clicks"),
        Input("planner-figure", "relayoutData"),
    ]
    scene_callback_states = [State("obstacle-layout-store", "data")] + [State(field["id"], "value") for field in ALL_PARAMETER_FIELDS]

    planner_callback_inputs = [
        Input("replan-button", "n_clicks"),
        Input("obstacle-layout-store", "data"),
        Input("scene-update-store", "data"),
    ] + [Input(field["id"], "value") for field in ALL_PARAMETER_FIELDS]

    @app.callback(scene_callback_outputs, scene_callback_inputs, scene_callback_states, prevent_initial_call=True)
    def update_scene_state(_randomize_clicks, relayout_data, obstacle_layout_state, *state_values):
        trigger = None
        if callback_context.triggered:
            trigger = callback_context.triggered[0]["prop_id"].split(".")[0]

        obstacle_layout = normalize_obstacle_layout(obstacle_layout_state)

        raw_values = {
            field["id"]: state_values[index]
            for index, field in enumerate(ALL_PARAMETER_FIELDS)
        }
        control_values = collect_control_values(raw_values)

        if trigger == "planner-figure":
            obstacle_layout, control_values, changed = parse_shape_edit_state(
                relayout_data,
                obstacle_layout,
                control_values,
            )
            if not changed:
                raise PreventUpdate

            return (
                obstacle_layout,
                control_values["start_x"],
                control_values["start_y"],
                control_values["goal_x"],
                control_values["goal_y"],
                {"source": "drag", "nonce": time.time_ns()},
            )

        if trigger == "randomize-button":
            (start_x, start_y), (goal_x, goal_y) = sample_start_goal(control_values["robot_radius"], obstacle_layout)
            control_values["start_x"] = round(start_x, 2)
            control_values["start_y"] = round(start_y, 2)
            control_values["goal_x"] = round(goal_x, 2)
            control_values["goal_y"] = round(goal_y, 2)

            return (
                obstacle_layout,
                control_values["start_x"],
                control_values["start_y"],
                control_values["goal_x"],
                control_values["goal_y"],
                {"source": "randomize", "nonce": time.time_ns()},
            )

        raise PreventUpdate

    @app.callback(
        [Output(accordion_item_id, "open") for accordion_item_id in ACCORDION_ITEM_IDS],
        Input("collapse-all-button", "n_clicks"),
        prevent_initial_call=True,
    )
    def collapse_all_accordions(_n_clicks):
        return [False] * len(ACCORDION_ITEM_IDS)

    @app.callback(planner_callback_outputs, planner_callback_inputs)
    def update_dashboard(_replan_clicks, obstacle_layout_state, _scene_update_state, *input_values):
        try:
            trigger_ids = {
                item["prop_id"].split(".")[0]
                for item in callback_context.triggered
                if item.get("prop_id")
            }
            raw_values = {
                field["id"]: input_values[index]
                for index, field in enumerate(ALL_PARAMETER_FIELDS)
            }
            control_values = collect_control_values(raw_values)
            obstacle_layout = normalize_obstacle_layout(obstacle_layout_state)

            if (
                "replan-button" not in trigger_ids
                and "scene-update-store" not in trigger_ids
                and not control_values["auto_replan"]
            ):
                raise PreventUpdate

            plan_result = compute_plan(control_values, obstacle_layout)
            figure = build_figure(plan_result, control_values, obstacle_layout)
            success_text = "Succeeded" if plan_result["success"] else "Failed"
            banner_text = (
                f"{success_text}: {plan_result['pose_count']} poses, "
                f"duration {format_metric(plan_result['duration'], ' s')}"
            )
            banner = build_status_banner(banner_text, plan_result["success"])
            if plan_result["command"] is None:
                command_text = "--"
            else:
                command_text = (
                    f"vx {plan_result['command'][0]:.2f} m/s, "
                    f"omega {plan_result['command'][1]:.2f} rad/s"
                )

            return (
                figure,
                banner,
                success_text,
                format_metric(plan_result["planning_time_ms"], " ms", precision=1),
                str(plan_result["pose_count"]),
                format_metric(plan_result["duration"], " s"),
                format_metric(plan_result["path_length"], " m"),
                format_metric(plan_result["min_clearance"], " m"),
                format_metric(plan_result["cost"]),
                command_text,
                "Yes" if plan_result["diverged"] else "No",
                plan_result["table_data"],
            )
        except PreventUpdate:
            raise
        except Exception as exc:
            raw_values = {
                field["id"]: input_values[index]
                for index, field in enumerate(ALL_PARAMETER_FIELDS)
            }
            control_values = collect_control_values(raw_values)
            obstacle_layout = normalize_obstacle_layout(obstacle_layout_state)
            empty_result = {
                "start": make_pose(
                    (control_values["start_x"], control_values["start_y"]),
                    (control_values["goal_x"], control_values["goal_y"]),
                ),
                "goal": make_goal_pose(
                    (control_values["goal_x"], control_values["goal_y"]),
                    (control_values["start_x"], control_values["start_y"]),
                ),
                "trajectory": np.zeros((0, 2), dtype=float),
                "segment_mid_times": [],
                "linear_speeds": [],
                "angular_speeds": [],
            }
            figure = build_figure(empty_result, control_values, obstacle_layout)
            banner = build_status_banner(f"Planning error: {exc}", False)
            return (
                figure,
                banner,
                "Error",
                "--",
                "--",
                "--",
                "--",
                "--",
                "--",
                "--",
                "--",
                [],
            )

    return app


def main():
    random.seed()
    app = create_app()
    host = os.environ.get("TEB_WEB_HOST", "127.0.0.1")
    port = int(os.environ.get("TEB_WEB_PORT", "8050"))
    app.run(host=host, port=port, debug=False)
    return 0


if __name__ == "__main__":
    sys.exit(main())