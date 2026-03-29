#!/usr/bin/env python3

import math
from pathlib import Path
import random
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle
from matplotlib.widgets import Button


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
ROBOT_RADIUS = 0.28
MIN_START_GOAL_DISTANCE = 5.0

OBSTACLE_LAYOUT = [
    (-2.4, -1.2, 0.55),
    (-1.0, 1.0, 0.6),
    (0.7, -0.2, 0.5),
    (2.0, 1.4, 0.55),
    (2.4, -1.5, 0.45),
]


def build_config():
    cfg = teb.TebConfig()
    cfg.robot_model = teb.CircularRobotFootprint(ROBOT_RADIUS)

    cfg.trajectory.dt_ref = 0.25
    cfg.trajectory.dt_hysteresis = 0.08
    cfg.trajectory.min_samples = 6
    cfg.trajectory.max_samples = 80
    cfg.trajectory.global_plan_overwrite_orientation = False

    cfg.robot.max_vel_x = 1.2
    cfg.robot.max_vel_x_backwards = 0.2
    cfg.robot.max_vel_theta = 1.2
    cfg.robot.acc_lim_x = 1.5
    cfg.robot.acc_lim_theta = 2.0

    cfg.obstacles.min_obstacle_dist = 0.3
    cfg.obstacles.inflation_dist = 0.55
    cfg.obstacles.obstacle_poses_affected = 20

    cfg.optim.no_inner_iterations = 6
    cfg.optim.no_outer_iterations = 4
    cfg.optim.weight_optimaltime = 1.0
    cfg.optim.weight_obstacle = 60.0
    cfg.optim.weight_inflation = 0.2
    cfg.optim.weight_kinematics_nh = 1000.0
    cfg.optim.weight_kinematics_forward_drive = 10.0

    cfg.checkParameters()
    return cfg


def build_obstacles():
    obstacles = teb.ObstacleContainer()
    for x, y, radius in OBSTACLE_LAYOUT:
        obstacles.append(teb.CircularObstacle(x, y, radius))
    return obstacles


def distance(point_a, point_b):
    return math.hypot(point_a[0] - point_b[0], point_a[1] - point_b[1])


def is_pose_collision_free(point):
    for obstacle_x, obstacle_y, radius in OBSTACLE_LAYOUT:
        clearance = radius + ROBOT_RADIUS + 0.2
        if distance(point, (obstacle_x, obstacle_y)) <= clearance:
            return False
    return True


def sample_position():
    x_margin = ROBOT_RADIUS + 0.2
    y_margin = ROBOT_RADIUS + 0.2
    for _ in range(1000):
        candidate = (
            random.uniform(X_LIMITS[0] + x_margin, X_LIMITS[1] - x_margin),
            random.uniform(Y_LIMITS[0] + y_margin, Y_LIMITS[1] - y_margin),
        )
        if is_pose_collision_free(candidate):
            return candidate
    raise RuntimeError("Failed to sample a collision-free pose.")


def sample_start_goal():
    for _ in range(1000):
        start = sample_position()
        goal = sample_position()
        if distance(start, goal) >= MIN_START_GOAL_DISTANCE:
            return start, goal
    raise RuntimeError("Failed to sample a valid start/goal pair.")


def make_pose(position, target):
    heading = math.atan2(target[1] - position[1], target[0] - position[0])
    return teb.PoseSE2(position[0], position[1], heading)


def make_goal_pose(position, start):
    heading = math.atan2(position[1] - start[1], position[0] - start[0])
    return teb.PoseSE2(position[0], position[1], heading)


def draw_heading(ax, pose, color):
    length = 0.45
    dx = length * math.cos(pose.theta())
    dy = length * math.sin(pose.theta())
    ax.arrow(
        pose.x(),
        pose.y(),
        dx,
        dy,
        width=0.03,
        head_width=0.16,
        head_length=0.18,
        color=color,
        length_includes_head=True,
        zorder=5,
    )


class InteractivePlannerDemo:
    def __init__(self):
        self.cfg = build_config()
        self.obstacles = build_obstacles()
        self.planner = teb.TebOptimalPlanner(self.cfg, self.obstacles)
        self.current_start = None
        self.current_goal = None

        self.figure, self.ax = plt.subplots(figsize=(10, 7))
        plt.subplots_adjust(bottom=0.18)

        button_ax = self.figure.add_axes([0.38, 0.05, 0.24, 0.07])
        self.random_button = Button(button_ax, "Random Start/Goal")
        self.random_button.on_clicked(self.on_randomize)

        self.status_text = self.figure.text(0.02, 0.03, "", fontsize=10)
        self.replan_with_random_endpoints()

    def extract_trajectory(self):
        poses = self.planner.teb().poses()
        if not poses:
            return np.zeros((0, 2))
        return np.array([[pose.x(), pose.y()] for pose in poses], dtype=float)

    def replan_with_random_endpoints(self):
        trajectory = np.zeros((0, 2))
        status = "Planning failed after multiple random samples"

        for attempt in range(1, 51):
            start_xy, goal_xy = sample_start_goal()
            start = make_pose(start_xy, goal_xy)
            goal = make_goal_pose(goal_xy, start_xy)

            self.planner.clearPlanner()
            planned = self.planner.plan(start, goal)

            self.current_start = start
            self.current_goal = goal

            if planned:
                trajectory = self.extract_trajectory()
                status = (
                    f"Planning succeeded with {len(trajectory)} trajectory poses "
                    f"after {attempt} sample(s)"
                )
                break

        self.draw_scene(trajectory, status)

    def draw_scene(self, trajectory, status):
        self.ax.clear()
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlim(*X_LIMITS)
        self.ax.set_ylim(*Y_LIMITS)
        self.ax.set_title("TEB standalone planner demo")
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)

        for obstacle_x, obstacle_y, radius in OBSTACLE_LAYOUT:
            obstacle_artist = Circle(
                (obstacle_x, obstacle_y),
                radius,
                facecolor="#d1495b",
                edgecolor="#8c1c13",
                linewidth=2,
                alpha=0.75,
                zorder=2,
            )
            self.ax.add_patch(obstacle_artist)

        if len(trajectory) > 0:
            self.ax.plot(
                trajectory[:, 0],
                trajectory[:, 1],
                color="#00798c",
                linewidth=2.5,
                label="TEB trajectory",
                zorder=3,
            )

        self.ax.scatter(
            [self.current_start.x()],
            [self.current_start.y()],
            color="#2a9d8f",
            s=90,
            label="start",
            zorder=4,
        )
        self.ax.scatter(
            [self.current_goal.x()],
            [self.current_goal.y()],
            color="#f4a261",
            s=90,
            label="goal",
            zorder=4,
        )
        draw_heading(self.ax, self.current_start, "#2a9d8f")
        draw_heading(self.ax, self.current_goal, "#f4a261")

        self.ax.legend(loc="upper left")
        self.status_text.set_text(status)
        self.figure.canvas.draw_idle()

    def on_randomize(self, _event):
        self.replan_with_random_endpoints()


def main():
    random.seed()
    demo = InteractivePlannerDemo()
    plt.show()
    return demo


if __name__ == "__main__":
    sys.exit(0 if main() is not None else 1)