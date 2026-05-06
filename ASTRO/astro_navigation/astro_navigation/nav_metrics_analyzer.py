#!/usr/bin/env python3
import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path


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


def load_runs(root_dir):
    root = Path(root_dir)
    runs = []

    for md_path in sorted(root.rglob("metadata.json")):
        run_dir = md_path.parent

        with open(md_path) as f:
            md = json.load(f)

        odom_path = run_dir / "odom.csv"
        plan_path = run_dir / "plan.csv"

        odom_points = read_csv_points(odom_path) if odom_path.exists() else []
        plan_points = read_csv_points(plan_path) if plan_path.exists() else []

        actual_length = path_length(odom_points)
        ref_length = path_length(plan_points)
        tr_mean, tr_rmse, tr_max = tracking_metrics(odom_points, plan_points)

        runs.append(
            {
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
            }
        )

    return runs


def write_runs_summary(runs, out_csv):
    fieldnames = [
        "run_name",
        "planner",
        "controller",
        "scenario",
        "success",
        "result_status",
        "planning_time_s",
        "navigation_time_s",
        "actual_path_length_m",
        "reference_path_length_m",
        "tracking_mean_m",
        "tracking_rmse_m",
        "tracking_max_m",
        "min_obstacle_distance_m",
        "odom_samples",
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
        "planner",
        "controller",
        "runs",
        "success_rate",
        "planning_time_mean_s",
        "navigation_time_mean_s",
        "actual_path_length_mean_m",
        "reference_path_length_mean_m",
        "tracking_mean_mean_m",
        "tracking_rmse_mean_m",
        "tracking_max_mean_m",
        "min_obstacle_distance_mean_m",
    ]

    with open(out_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for (planner, controller), items in sorted(grouped.items()):
            n = len(items)
            writer.writerow(
                {
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
                    "min_obstacle_distance_mean_m": mean(
                        r["min_obstacle_distance_m"] for r in items
                    ),
                }
            )


def main():
    ap = argparse.ArgumentParser(
        description="Analiza metrika nad već spremljenim i već korigiranim runovima."
    )
    ap.add_argument(
        "--input_dir",
        required=True,
        help="Root direktorij koji sadrži run direktorije.",
    )
    args = ap.parse_args()

    root = Path(args.input_dir)
    runs = load_runs(root)

    write_runs_summary(runs, root / "runs_summary.csv")
    write_combo_summary(runs, root / "combo_summary.csv")

    print(f"Processed {len(runs)} runs in {root}")
    print(f"Wrote: {root / 'runs_summary.csv'}")
    print(f"Wrote: {root / 'combo_summary.csv'}")


if __name__ == "__main__":
    main()
