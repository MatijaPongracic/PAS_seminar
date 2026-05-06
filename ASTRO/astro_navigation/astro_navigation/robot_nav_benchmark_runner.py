#!/usr/bin/env python3
import csv
import json
import math
import os
import time
from pathlib import Path

import numpy as np
import pandas as pd
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def get_metrics_dir(metrics_subdir: str) -> Path:
    env_root = os.environ.get("WORKSPACE_ROOT")
    if env_root:
        return Path(env_root).expanduser().resolve() / "metrics" / metrics_subdir

    candidates = []

    for var in ("COLCON_PREFIX_PATH", "AMENT_PREFIX_PATH"):
        for entry in os.environ.get(var, "").split(":"):
            if not entry:
                continue
            p = Path(entry).expanduser().resolve()
            candidates.append(p.parent if p.name == "install" else p)

    cwd = Path.cwd().resolve()
    here = Path(__file__).resolve()
    candidates.extend([cwd, *cwd.parents, here.parent, *here.parents])

    seen = set()
    for candidate in candidates:
        candidate = candidate.resolve()
        if candidate in seen:
            continue
        seen.add(candidate)

        metrics_root = candidate / "metrics"
        if metrics_root.exists():
            return metrics_root / metrics_subdir

    raise RuntimeError(
        f"Ne mogu pronaći workspace root za metrics/{metrics_subdir}. "
        "Postavi WORKSPACE_ROOT, npr. "
        "'export WORKSPACE_ROOT=/home/$USER/ime_workspacea'"
    )


def quat_to_yaw(x, y, z, w):
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def duration_to_sec(duration_msg):
    return float(duration_msg.sec) + float(duration_msg.nanosec) * 1e-9


def status_to_string(status_code):
    mapping = {
        0: "UNKNOWN",
        1: "ACCEPTED",
        2: "EXECUTING",
        3: "CANCELING",
        4: "SUCCEEDED",
        5: "CANCELED",
        6: "ABORTED",
    }
    return mapping.get(int(status_code), f"STATUS_{status_code}")


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def compute_yaw(x1, y1, x2, y2):
    return math.atan2(y2 - y1, x2 - x1)


def fill_plan_yaw_csv(csv_path: Path):
    rows = []

    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames
        required = {"x", "y", "yaw"}

        if not fieldnames or not required.issubset(fieldnames):
            raise ValueError("plan.csv mora sadržavati stupce x, y, yaw")

        for row in reader:
            rows.append(row)

    if len(rows) < 2:
        return False

    for i in range(len(rows) - 1):
        x1 = float(rows[i]["x"])
        y1 = float(rows[i]["y"])
        x2 = float(rows[i + 1]["x"])
        y2 = float(rows[i + 1]["y"])
        rows[i]["yaw"] = f"{compute_yaw(x1, y1, x2, y2):.12f}"

    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    return True


def cumulative_arclength(xy):
    d = np.sqrt(np.sum(np.diff(xy, axis=0) ** 2, axis=1))
    s = np.concatenate([[0.0], np.cumsum(d)])
    return s


def resample_by_arclength(xy, n_points):
    s = cumulative_arclength(xy)
    total = float(s[-1])

    if total == 0.0:
        return (
            np.repeat(xy[:1], n_points, axis=0),
            np.linspace(0.0, 1.0, n_points),
            total,
        )

    q = np.linspace(0.0, total, n_points)
    xr = np.interp(q, s, xy[:, 0])
    yr = np.interp(q, s, xy[:, 1])
    return np.column_stack([xr, yr]), q / total, total


def direction_angle(xy, lookahead=20):
    if len(xy) < 2:
        return 0.0
    j = min(max(1, lookahead), len(xy) - 1)
    v = xy[j] - xy[0]
    return math.atan2(v[1], v[0])


def rotation_matrix(theta):
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=float)


def apply_anchored_rotation(xy, theta, plan_start, odom_start):
    R = rotation_matrix(theta)
    t = np.asarray(plan_start, dtype=float) - R @ np.asarray(odom_start, dtype=float)
    out = (R @ xy.T).T + t
    return out, R, t


def weighted_rmse(a, b, w):
    err2 = np.sum((a - b) ** 2, axis=1)
    return float(np.sqrt(np.sum(w * err2) / np.sum(w)))


def objective(theta, odom_rs, plan_rs, w):
    aligned, _, _ = apply_anchored_rotation(odom_rs, theta, plan_rs[0], odom_rs[0])
    err2 = np.sum((aligned - plan_rs) ** 2, axis=1)
    return float(np.sum(w * err2) / np.sum(w))


def search_best_theta(
    odom_rs,
    plan_rs,
    w,
    initial_theta,
    coarse_deg=0.25,
    fine_window_deg=2.0,
    fine_deg=0.01,
):
    coarse = np.deg2rad(coarse_deg)
    grid = np.arange(-math.pi, math.pi + 0.5 * coarse, coarse)
    vals = np.array([objective(th, odom_rs, plan_rs, w) for th in grid])
    best_idx = int(np.argmin(vals))
    best_theta = float(grid[best_idx])

    center = best_theta if np.isfinite(best_theta) else initial_theta
    half = math.radians(fine_window_deg)
    step = math.radians(fine_deg)
    fine_grid = np.arange(center - half, center + half + 0.5 * step, step)
    fine_vals = np.array([objective(th, odom_rs, plan_rs, w) for th in fine_grid])
    best_theta = float(fine_grid[int(np.argmin(fine_vals))])
    return wrap_angle(best_theta)


def estimate_transform(
    plan_xy,
    odom_xy,
    n_points=800,
    start_weight=50.0,
    anchor_fraction=0.08,
    lookahead=20,
):
    odom_rs, _, odom_len = resample_by_arclength(odom_xy, n_points)
    plan_rs, _, plan_len = resample_by_arclength(plan_xy, n_points)

    th0 = wrap_angle(direction_angle(plan_rs, lookahead) - direction_angle(odom_rs, lookahead))
    u = np.linspace(0.0, 1.0, n_points)
    w = 1.0 + start_weight * np.exp(-u / max(anchor_fraction, 1e-6))
    theta = search_best_theta(odom_rs, plan_rs, w, th0)

    aligned_rs, R, t = apply_anchored_rotation(odom_rs, theta, plan_rs[0], odom_rs[0])
    info = {
        "theta_rad": float(theta),
        "theta_deg": float(math.degrees(theta)),
        "tx": float(t[0]),
        "ty": float(t[1]),
        "start_weight": float(start_weight),
        "anchor_fraction": float(anchor_fraction),
        "n_points": int(n_points),
        "odom_length_m": float(odom_len),
        "plan_length_m": float(plan_len),
        "resampled_weighted_rmse_m": weighted_rmse(aligned_rs, plan_rs, w),
        "resampled_unweighted_rmse_m": float(
            np.sqrt(np.mean(np.sum((aligned_rs - plan_rs) ** 2, axis=1)))
        ),
    }
    return theta, R, t, info


def transform_odom_dataframe(df, theta, R, t):
    out = df.copy()
    xy = out[["x", "y"]].to_numpy(float)
    xy_new = (R @ xy.T).T + t
    out["x"] = xy_new[:, 0]
    out["y"] = xy_new[:, 1]

    if "yaw" in out.columns:
        out["yaw"] = out["yaw"].astype(float).map(lambda a: wrap_angle(a + theta))

    return out


def overwrite_odom_with_aligned_plan(
    run_dir,
    n_points=800,
    start_weight=50.0,
    anchor_fraction=0.08,
):
    run_dir = Path(run_dir)
    plan_path = run_dir / "plan.csv"
    odom_path = run_dir / "odom.csv"

    if not plan_path.exists() or not odom_path.exists():
        return None

    plan_df = pd.read_csv(plan_path)
    odom_df = pd.read_csv(odom_path)

    for col in ["x", "y"]:
        if col not in plan_df.columns or col not in odom_df.columns:
            raise ValueError(f"{run_dir}: i plan.csv i odom.csv moraju imati kolone x i y")

    if len(plan_df) == 0 or len(odom_df) == 0:
        return None

    plan_xy = plan_df[["x", "y"]].to_numpy(float)
    odom_xy = odom_df[["x", "y"]].to_numpy(float)

    raw_start_dist = float(np.linalg.norm(plan_xy[0] - odom_xy[0]))
    theta, R, t, info = estimate_transform(
        plan_xy,
        odom_xy,
        n_points=n_points,
        start_weight=start_weight,
        anchor_fraction=anchor_fraction,
    )

    aligned_df = transform_odom_dataframe(odom_df, theta, R, t)
    aligned_df.to_csv(odom_path, index=False)

    aligned_xy = aligned_df[["x", "y"]].to_numpy(float)
    aligned_start_dist = float(np.linalg.norm(plan_xy[0] - aligned_xy[0]))

    info.update(
        {
            "raw_start_distance_m": raw_start_dist,
            "aligned_start_distance_m": aligned_start_dist,
            "plan_input": str(plan_path),
            "odom_overwritten": str(odom_path),
        }
    )
    return info


class NavBenchmarkRunner(Node):
    def __init__(self):
        super().__init__("nav_benchmark_runner")

        self.declare_parameter(
            "output_dir",
            str(get_metrics_dir("metrics_robot"))
        )

        self.declare_parameter("run_name", "run01")
        self.declare_parameter("planner_id", "GridBased")
        self.declare_parameter("planner_label", "astar")
        self.declare_parameter("controller_label", "mppi")
        self.declare_parameter("scenario", "default")

        self.declare_parameter("goal_x", 1.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_yaw", 0.0)

        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("compute_path_action", "compute_path_to_pose")
        self.declare_parameter("navigate_action", "navigate_to_pose")

        self.declare_parameter("fill_plan_yaw", True)
        self.declare_parameter("align_odom_to_plan", True)
        self.declare_parameter("align_n_points", 800)
        self.declare_parameter("align_start_weight", 50.0)
        self.declare_parameter("align_anchor_fraction", 0.08)

        self.output_dir = Path(self.get_parameter("output_dir").value)
        self.run_name = str(self.get_parameter("run_name").value)
        self.planner_id = str(self.get_parameter("planner_id").value)
        self.planner_label = str(self.get_parameter("planner_label").value)
        self.controller_label = str(self.get_parameter("controller_label").value)
        self.scenario = str(self.get_parameter("scenario").value)

        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.goal_yaw = float(self.get_parameter("goal_yaw").value)

        self.fill_plan_yaw = bool(self.get_parameter("fill_plan_yaw").value)
        self.align_odom_to_plan = bool(self.get_parameter("align_odom_to_plan").value)
        self.align_n_points = int(self.get_parameter("align_n_points").value)
        self.align_start_weight = float(self.get_parameter("align_start_weight").value)
        self.align_anchor_fraction = float(self.get_parameter("align_anchor_fraction").value)

        self.run_dir = self.output_dir / self.run_name
        self.run_dir.mkdir(parents=True, exist_ok=True)

        odom_topic = str(self.get_parameter("odom_topic").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        compute_path_action = str(self.get_parameter("compute_path_action").value)
        navigate_action = str(self.get_parameter("navigate_action").value)

        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_cb,
            50,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_cb,
            50,
        )

        self.compute_client = ActionClient(
            self,
            ComputePathToPose,
            compute_path_action,
        )

        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            navigate_action,
        )

        self.current_pose = None
        self.odom_records = []
        self.min_obstacle_distance = float("inf")
        self.run_active = False

    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        self.current_pose = (x, y, yaw, stamp)

        if self.run_active:
            self.odom_records.append([stamp, x, y, yaw])

    def scan_cb(self, msg: LaserScan):
        if not self.run_active:
            return

        finite_ranges = [r for r in msg.ranges if math.isfinite(r)]
        if finite_ranges:
            self.min_obstacle_distance = min(
                self.min_obstacle_distance,
                min(finite_ranges),
            )

    def wait_for_odom(self):
        while rclpy.ok() and self.current_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        z, w = yaw_to_quat(yaw)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        return pose

    def save_plan(self, path_msg):
        with open(self.run_dir / "plan.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["idx", "x", "y", "yaw"])

            for i, pose_stamped in enumerate(path_msg.poses):
                p = pose_stamped.pose.position
                q = pose_stamped.pose.orientation
                yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
                writer.writerow([i, p.x, p.y, yaw])

    def save_odom(self):
        with open(self.run_dir / "odom.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["t", "x", "y", "yaw"])
            writer.writerows(self.odom_records)

    def save_metadata(
        self,
        planning_time_s,
        nav_time_s,
        result_status,
        plan_yaw_filled=False,
        odom_alignment=None,
        odom_alignment_error=None,
    ):
        md = {
            "run_name": self.run_name,
            "planner": self.planner_label,
            "controller": self.controller_label,
            "planner_id": self.planner_id,
            "scenario": self.scenario,
            "goal": {
                "x": self.goal_x,
                "y": self.goal_y,
                "yaw": self.goal_yaw,
            },
            "planning_time_s": planning_time_s,
            "navigation_time_s": nav_time_s,
            "result_status_code": int(result_status),
            "result_status": status_to_string(result_status),
            "success": int(result_status) == 4,
            "min_obstacle_distance_m": (
                None if math.isinf(self.min_obstacle_distance)
                else self.min_obstacle_distance
            ),
            "odom_samples": len(self.odom_records),
            "plan_yaw_filled": plan_yaw_filled,
            "odom_alignment": odom_alignment,
            "odom_alignment_error": odom_alignment_error,
        }

        with open(self.run_dir / "metadata.json", "w") as f:
            json.dump(md, f, indent=2)

    def compute_path(self):
        self.get_logger().info("Waiting for compute_path_to_pose action server...")
        self.compute_client.wait_for_server()

        goal = ComputePathToPose.Goal()
        goal.goal = self.make_pose(self.goal_x, self.goal_y, self.goal_yaw)
        goal.planner_id = self.planner_id
        goal.use_start = False

        send_future = self.compute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("ComputePathToPose goal was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()

        if wrapped_result is None:
            raise RuntimeError("ComputePathToPose returned no result")

        result = wrapped_result.result
        return result.path, duration_to_sec(result.planning_time)

    def navigate(self):
        self.get_logger().info("Waiting for navigate_to_pose action server...")
        self.navigate_client.wait_for_server()

        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(self.goal_x, self.goal_y, self.goal_yaw)

        send_future = self.navigate_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("NavigateToPose goal was rejected")

        self.run_active = True
        nav_t0 = time.perf_counter()

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        nav_t1 = time.perf_counter()
        self.run_active = False

        wrapped_result = result_future.result()
        if wrapped_result is None:
            raise RuntimeError("NavigateToPose returned no result")

        return wrapped_result.status, nav_t1 - nav_t0

    def run(self):
        self.wait_for_odom()

        self.get_logger().info(f"Computing path with planner_id={self.planner_id}")
        path_msg, planning_time_s = self.compute_path()
        self.save_plan(path_msg)

        plan_yaw_filled = False
        if self.fill_plan_yaw:
            try:
                plan_yaw_filled = fill_plan_yaw_csv(self.run_dir / "plan.csv")
                if plan_yaw_filled:
                    self.get_logger().info("Yaw stupac u plan.csv je matematički popunjen.")
                else:
                    self.get_logger().warning(
                        "Yaw stupac u plan.csv nije popunjen jer plan ima manje od 2 retka."
                    )
            except Exception as exc:
                self.get_logger().warning(f"Popunjavanje yaw stupca u plan.csv nije uspjelo: {exc}")

        self.get_logger().info("Sending NavigateToPose goal")
        result_status, nav_time_s = self.navigate()

        self.save_odom()

        odom_alignment = None
        odom_alignment_error = None

        if self.align_odom_to_plan:
            try:
                odom_alignment = overwrite_odom_with_aligned_plan(
                    self.run_dir,
                    n_points=self.align_n_points,
                    start_weight=self.align_start_weight,
                    anchor_fraction=self.align_anchor_fraction,
                )
                if odom_alignment is not None:
                    self.get_logger().info(
                        "Odom korekcija završena: "
                        f"theta={odom_alignment['theta_deg']:.4f} deg, "
                        f"raw_start={odom_alignment['raw_start_distance_m']:.4f} m, "
                        f"aligned_start={odom_alignment['aligned_start_distance_m']:.4f} m"
                    )
                else:
                    self.get_logger().warning(
                        "Odom korekcija je preskočena jer nedostaje plan.csv ili odom.csv, "
                        "ili je jedna od datoteka prazna."
                    )
            except Exception as exc:
                odom_alignment_error = str(exc)
                self.get_logger().warning(f"Odom korekcija nije uspjela: {exc}")

        self.save_metadata(
            planning_time_s,
            nav_time_s,
            result_status,
            plan_yaw_filled=plan_yaw_filled,
            odom_alignment=odom_alignment,
            odom_alignment_error=odom_alignment_error,
        )

        self.get_logger().info(f"Saved run to: {self.run_dir}")
        self.get_logger().info(f"Result: {status_to_string(result_status)}")


def main():
    rclpy.init()
    node = NavBenchmarkRunner()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
