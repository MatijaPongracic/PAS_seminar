#!/usr/bin/env python3
import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np
import pandas as pd


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def cumulative_arclength(xy):
    d = np.sqrt(np.sum(np.diff(xy, axis=0) ** 2, axis=1))
    s = np.concatenate([[0.0], np.cumsum(d)])
    return s


def resample_by_arclength(xy, n_points):
    s = cumulative_arclength(xy)
    total = float(s[-1])
    if total == 0.0:
        return np.repeat(xy[:1], n_points, axis=0), np.linspace(0.0, 1.0, n_points), total
    q = np.linspace(0.0, total, n_points)
    xr = np.interp(q, s, xy[:, 0])
    yr = np.interp(q, s, xy[:, 1])
    return np.column_stack([xr, yr]), q / total, total


def direction_angle(xy, lookahead=20):
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


def estimate_transform(plan_xy, odom_xy, n_points=800, start_weight=50.0, anchor_fraction=0.08, lookahead=20):
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
        "resampled_unweighted_rmse_m": float(np.sqrt(np.mean(np.sum((aligned_rs - plan_rs) ** 2, axis=1)))),
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

    info.update({
        "raw_start_distance_m": raw_start_dist,
        "aligned_start_distance_m": aligned_start_dist,
        "plan_input": str(plan_path),
        "odom_overwritten": str(odom_path),
    })
    return info


def read_csv_points(path, x_col="x", y_col="y"):
    pts = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            pts.append((float(row[x_col]), float(row[y_col])))
    return pts


def path_length(points):
    if len(points) < 2:
        return 0.0
    total = 0.0
    for i in range(len(points) - 1):
        x1, y1 = points[i]
        x2, y2 = points[i + 1]
        total += math.hypot(x2 - x1, y2 - y1)
    return total


def nearest_distance(point, path_points):
    x, y = point
    return min(math.hypot(x - px, y - py) for px, py in path_points)


def tracking_metrics(odom_points, plan_points):
    if not odom_points or not plan_points:
        return None, None, None
    ds = [nearest_distance(p, plan_points) for p in odom_points]
    mean_d = sum(ds) / len(ds)
    rmse = math.sqrt(sum(d * d for d in ds) / len(ds))
    max_d = max(ds)
    return mean_d, rmse, max_d


def mean(values):
    vals = [v for v in values if v is not None]
    if not vals:
        return None
    return sum(vals) / len(vals)


def load_runs(root_dir, n_points=800, start_weight=50.0, anchor_fraction=0.08):
    root = Path(root_dir)
    runs = []

    for md_path in sorted(root.rglob("metadata.json")):
        run_dir = md_path.parent

        overwrite_odom_with_aligned_plan(
            run_dir,
            n_points=n_points,
            start_weight=start_weight,
            anchor_fraction=anchor_fraction,
        )

        with open(md_path) as f:
            md = json.load(f)

        odom_path = run_dir / "odom.csv"
        plan_path = run_dir / "plan.csv"

        odom_points = read_csv_points(odom_path) if odom_path.exists() else []
        plan_points = read_csv_points(plan_path) if plan_path.exists() else []

        actual_length = path_length(odom_points)
        ref_length = path_length(plan_points)
        tr_mean, tr_rmse, tr_max = tracking_metrics(odom_points, plan_points)

        runs.append({
            "run_name": md.get("run_name"),
            "planner": md.get("planner"),
            "controller": md.get("controller"),
            "scenario": md.get("scenario"),
            "success": int(bool(md.get("success"))),
            "result_status": md.get("result_status"),
            "planning_time_s": md.get("planning_time_s"),
            "navigation_time_s": md.get("navigation_time_s"),
            "actual_path_length_m": actual_length,
            "reference_path_length_m": ref_length,
            "tracking_mean_m": tr_mean,
            "tracking_rmse_m": tr_rmse,
            "tracking_max_m": tr_max,
            "min_obstacle_distance_m": md.get("min_obstacle_distance_m"),
            "odom_samples": md.get("odom_samples"),
        })

    return runs


def write_runs_summary(runs, out_csv):
    fieldnames = [
        "run_name", "planner", "controller", "scenario", "success", "result_status",
        "planning_time_s", "navigation_time_s",
        "actual_path_length_m", "reference_path_length_m",
        "tracking_mean_m", "tracking_rmse_m", "tracking_max_m",
        "min_obstacle_distance_m", "odom_samples"
    ]
    with open(out_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in runs:
            writer.writerow(r)


def write_combo_summary(runs, out_csv):
    grouped = defaultdict(list)
    for r in runs:
        grouped[(r["planner"], r["controller"])].append(r)

    fieldnames = [
        "planner", "controller", "runs", "success_rate",
        "planning_time_mean_s", "navigation_time_mean_s",
        "actual_path_length_mean_m", "reference_path_length_mean_m",
        "tracking_mean_mean_m", "tracking_rmse_mean_m", "tracking_max_mean_m",
        "min_obstacle_distance_mean_m"
    ]

    with open(out_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for (planner, controller), items in sorted(grouped.items()):
            n = len(items)
            writer.writerow({
                "planner": planner,
                "controller": controller,
                "runs": n,
                "success_rate": sum(r["success"] for r in items) / n if n else None,
                "planning_time_mean_s": mean(r["planning_time_s"] for r in items),
                "navigation_time_mean_s": mean(r["navigation_time_s"] for r in items),
                "actual_path_length_mean_m": mean(r["actual_path_length_m"] for r in items),
                "reference_path_length_mean_m": mean(r["reference_path_length_m"] for r in items),
                "tracking_mean_mean_m": mean(r["tracking_mean_m"] for r in items),
                "tracking_rmse_mean_m": mean(r["tracking_rmse_m"] for r in items),
                "tracking_max_mean_m": mean(r["tracking_max_m"] for r in items),
                "min_obstacle_distance_mean_m": mean(r["min_obstacle_distance_m"] for r in items),
            })


def main():
    ap = argparse.ArgumentParser(
        description="Prvo poravnaj/overrideaj odom.csv u svakom run direktoriju, zatim odradi standardnu analizu metrika."
    )
    ap.add_argument("--input_dir", required=True, help="Root direktorij koji sadrži run direktorije.")
    ap.add_argument("--n_points", type=int, default=800, help="Broj točaka za resampling pri poravnanju.")
    ap.add_argument("--start_weight", type=float, default=50.0, help="Težina početka trajektorije pri poravnanju.")
    ap.add_argument("--anchor_fraction", type=float, default=0.08, help="Koliki početni dio nosi pojačanu težinu.")
    args = ap.parse_args()

    root = Path(args.input_dir)
    runs = load_runs(
        root,
        n_points=args.n_points,
        start_weight=args.start_weight,
        anchor_fraction=args.anchor_fraction,
    )

    write_runs_summary(runs, root / "runs_summary.csv")
    write_combo_summary(runs, root / "combo_summary.csv")

    print(f"Processed {len(runs)} runs in {root}")
    print(f"Wrote: {root / 'runs_summary.csv'}")
    print(f"Wrote: {root / 'combo_summary.csv'}")


if __name__ == "__main__":
    main()
