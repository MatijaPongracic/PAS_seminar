#!/usr/bin/env python3
import argparse
import csv
import io
import json
import math
import re
import shutil
import subprocess
import zipfile
from collections import defaultdict
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import gridspec
from matplotlib.collections import LineCollection

import rclpy
from rclpy.node import Node


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def interp_angle(a0: float, a1: float, u: float) -> float:
    return wrap_angle(a0 + u * wrap_angle(a1 - a0))


def yaw_to_quat(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def mean(values):
    vals = [v for v in values if v is not None]
    if not vals:
        return None
    return sum(vals) / len(vals)


def stddev(values):
    vals = [v for v in values if v is not None]
    if len(vals) < 2:
        return 0.0 if vals else None
    m = sum(vals) / len(vals)
    return math.sqrt(sum((v - m) ** 2 for v in vals) / len(vals))


def project_point_to_segment(px, py, ax, ay, bx, by):
    abx = bx - ax
    aby = by - ay
    ab2 = abx * abx + aby * aby

    if ab2 <= 1e-12:
        qx, qy = ax, ay
        u = 0.0
        dist2 = (px - qx) ** 2 + (py - qy) ** 2
        seg_len = 0.0
        return qx, qy, u, dist2, seg_len

    apx = px - ax
    apy = py - ay
    u = (apx * abx + apy * aby) / ab2
    u = max(0.0, min(1.0, u))

    qx = ax + u * abx
    qy = ay + u * aby
    dist2 = (px - qx) ** 2 + (py - qy) ** 2
    seg_len = math.sqrt(ab2)
    return qx, qy, u, dist2, seg_len


def cumulative_arclength(points_xy):
    s = [0.0]
    for i in range(len(points_xy) - 1):
        x1, y1 = points_xy[i]
        x2, y2 = points_xy[i + 1]
        s.append(s[-1] + math.hypot(x2 - x1, y2 - y1))
    return s


def load_plan(plan_path: Path):
    rows = []
    with open(plan_path, newline="") as f:
        reader = csv.DictReader(f)
        required = {"x", "y", "yaw"}
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise ValueError(f"{plan_path}: plan.csv mora imati kolone x, y, yaw")
        for row in reader:
            rows.append({
                "x": float(row["x"]),
                "y": float(row["y"]),
                "yaw": float(row["yaw"]),
            })
    return rows


def load_odom(odom_path: Path):
    rows = []
    with open(odom_path, newline="") as f:
        reader = csv.DictReader(f)
        required = {"t", "x", "y", "yaw"}
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise ValueError(f"{odom_path}: odom.csv mora imati kolone t, x, y, yaw")
        for row in reader:
            rows.append({
                "t": float(row["t"]),
                "x": float(row["x"]),
                "y": float(row["y"]),
                "yaw": float(row["yaw"]),
            })
    return rows


def write_csv(path: Path, fieldnames, rows):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_tum(path: Path, rows):
    with open(path, "w") as f:
        for r in rows:
            qx, qy, qz, qw = yaw_to_quat(float(r["yaw"]))
            f.write(
                f"{float(r['t']):.9f} "
                f"{float(r['x']):.12f} "
                f"{float(r['y']):.12f} "
                f"0.000000000000 "
                f"{qx:.12f} {qy:.12f} {qz:.12f} {qw:.12f}\n"
            )


def find_run_dirs(root_dir: Path):
    run_dirs = []
    for md_path in sorted(root_dir.rglob("metadata.json")):
        run_dir = md_path.parent
        if (run_dir / "plan.csv").exists() and (run_dir / "odom.csv").exists():
            run_dirs.append(run_dir)
    return run_dirs


def build_plan_time(plan_rows, odom_rows):
    if not plan_rows or not odom_rows:
        return []

    if len(plan_rows) == 1:
        p = plan_rows[0]
        return [{"t": od["t"], "x": p["x"], "y": p["y"], "yaw": p["yaw"]} for od in odom_rows]

    plan_xy = [(r["x"], r["y"]) for r in plan_rows]
    s_prefix = cumulative_arclength(plan_xy)

    out = []
    prev_seg_idx = 0
    prev_s = 0.0

    for od in odom_rows:
        px, py = od["x"], od["y"]
        best = None

        for i in range(prev_seg_idx, len(plan_rows) - 1):
            ax, ay = plan_rows[i]["x"], plan_rows[i]["y"]
            bx, by = plan_rows[i + 1]["x"], plan_rows[i + 1]["y"]

            qx, qy, u, dist2, seg_len = project_point_to_segment(px, py, ax, ay, bx, by)
            s_here = s_prefix[i] + u * seg_len

            if s_here + 1e-12 < prev_s:
                continue

            cand = (dist2, s_here, i, u, qx, qy)
            if best is None or cand[0] < best[0]:
                best = cand

        if best is None:
            i = len(plan_rows) - 2
            qx = plan_rows[-1]["x"]
            qy = plan_rows[-1]["y"]
            u = 1.0
            s_here = s_prefix[-1]
        else:
            _, s_here, i, u, qx, qy = best

        yaw0 = plan_rows[i]["yaw"]
        yaw1 = plan_rows[i + 1]["yaw"]
        yaw = interp_angle(yaw0, yaw1, u)

        out.append({
            "t": od["t"],
            "x": qx,
            "y": qy,
            "yaw": yaw,
        })

        prev_seg_idx = max(prev_seg_idx, i)
        prev_s = max(prev_s, s_here)

    return out


def parse_evo_stats(text: str):
    stats = {
        "rmse": None,
        "mean": None,
        "median": None,
        "std": None,
        "min": None,
        "max": None,
        "sse": None,
    }

    patterns = {
        "rmse": r"\brmse\s+([0-9eE+\-.]+)",
        "mean": r"\bmean\s+([0-9eE+\-.]+)",
        "median": r"\bmedian\s+([0-9eE+\-.]+)",
        "std": r"\bstd\s+([0-9eE+\-.]+)",
        "min": r"\bmin\s+([0-9eE+\-.]+)",
        "max": r"\bmax\s+([0-9eE+\-.]+)",
        "sse": r"\bsse\s+([0-9eE+\-.]+)",
    }

    for key, pattern in patterns.items():
        m = re.search(pattern, text)
        if m:
            stats[key] = float(m.group(1))

    return stats


def _load_json_from_zip(zf: zipfile.ZipFile, suffix: str):
    for name in zf.namelist():
        if name.endswith(suffix):
            with zf.open(name) as f:
                return json.load(f)
    return None


def _load_npy_from_zip(zf: zipfile.ZipFile, suffix: str):
    for name in zf.namelist():
        if name.endswith(suffix):
            with zf.open(name) as f:
                return np.load(io.BytesIO(f.read()), allow_pickle=True)
    return None


def _find_error_array_in_zip(zf: zipfile.ZipFile):
    preferred = [
        "error_array.npy",
        "errors.npy",
    ]
    for suffix in preferred:
        arr = _load_npy_from_zip(zf, suffix)
        if arr is not None:
            return np.asarray(arr, dtype=float)

    for name in zf.namelist():
        if name.endswith(".npy") and "error" in name.lower():
            with zf.open(name) as f:
                return np.load(io.BytesIO(f.read()), allow_pickle=True).astype(float)

    return None


def _find_timestamps_in_zip(zf: zipfile.ZipFile):
    for suffix in ["seconds_from_start.npy", "timestamps.npy"]:
        arr = _load_npy_from_zip(zf, suffix)
        if arr is not None:
            return np.asarray(arr, dtype=float)
    return None


def _find_traj_from_zip(zf: zipfile.ZipFile, traj_key_hint=("estimate", "est", "traj_est")):
    traj_json = None
    for name in zf.namelist():
        if name.endswith(".json") and "traj" in name.lower():
            with zf.open(name) as f:
                try:
                    obj = json.load(f)
                except Exception:
                    continue
                if isinstance(obj, dict):
                    traj_json = obj
                    break

    if traj_json is None:
        for name in zf.namelist():
            if name.endswith(".json"):
                with zf.open(name) as f:
                    try:
                        obj = json.load(f)
                    except Exception:
                        continue
                    if isinstance(obj, dict) and "trajectories" in obj:
                        traj_json = obj
                        break

    if traj_json is None:
        return None, None

    if "trajectories" in traj_json and isinstance(traj_json["trajectories"], dict):
        trajs = traj_json["trajectories"]
    else:
        trajs = traj_json

    for hint in traj_key_hint:
        for key, val in trajs.items():
            if hint.lower() in str(key).lower():
                return key, val

    if len(trajs) >= 1:
        key = list(trajs.keys())[-1]
        return key, trajs[key]

    return None, None


def _traj_xy_from_obj(traj_obj):
    if traj_obj is None:
        return None, None

    if isinstance(traj_obj, dict):
        for pos_key in ["positions_xyz", "positions", "xyz"]:
            if pos_key in traj_obj:
                arr = np.asarray(traj_obj[pos_key], dtype=float)
                if arr.ndim == 2 and arr.shape[1] >= 2:
                    return arr[:, 0], arr[:, 1]

        if "poses_se3" in traj_obj:
            poses = np.asarray(traj_obj["poses_se3"], dtype=float)
            if poses.ndim == 3 and poses.shape[1:] == (4, 4):
                return poses[:, 0, 3], poses[:, 1, 3]

    return None, None


def _fallback_xy_from_tum(tum_path: Path):
    xs, ys = [], []
    with open(tum_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            xs.append(float(parts[1]))
            ys.append(float(parts[2]))
    if not xs:
        return None, None
    return np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)


def generate_metric_report_png(
    zip_path: Path,
    metric_name: str,
    out_png: Path,
    tum_fallback: Path,
    stats_dict: dict,
    align_used: bool,
):
    if not zip_path.exists():
        return False

    with zipfile.ZipFile(zip_path, "r") as zf:
        err = _find_error_array_in_zip(zf)
        time_arr = _find_timestamps_in_zip(zf)

        _, traj_obj = _find_traj_from_zip(zf)
        xs, ys = _traj_xy_from_obj(traj_obj)

    if err is None or len(err) == 0:
        return False

    err = np.asarray(err, dtype=float).reshape(-1)

    if time_arr is None or len(time_arr) != len(err):
        time_arr = np.arange(len(err), dtype=float)

    if xs is None or ys is None or len(xs) != len(err) or len(ys) != len(err):
        xs, ys = _fallback_xy_from_tum(tum_fallback)

    if xs is None or ys is None:
        xs = np.arange(len(err), dtype=float)
        ys = np.zeros_like(xs)

    n = min(len(err), len(time_arr), len(xs), len(ys))
    err = err[:n]
    time_arr = time_arr[:n]
    xs = xs[:n]
    ys = ys[:n]

    plt.style.use("seaborn-v0_8")
    fig = plt.figure(figsize=(14, 10), constrained_layout=False)
    gs = gridspec.GridSpec(
        2, 2,
        width_ratios=[3.2, 1.4],
        height_ratios=[1.1, 1.5],
        wspace=0.28,
        hspace=0.28
    )

    ax_top = fig.add_subplot(gs[0, 0])
    ax_tbl = fig.add_subplot(gs[0, 1])
    ax_traj = fig.add_subplot(gs[1, :])

    color_raw = "#8f8f8f"
    color_rmse = "#4C72B0"
    color_median = "#55A868"
    color_mean = "#C44E52"
    color_std = "#8172B2"

    ax_top.plot(time_arr, err, color=color_raw, linewidth=0.9, alpha=0.85, label=metric_name.lower())
    if stats_dict.get("rmse") is not None:
        ax_top.axhline(stats_dict["rmse"], color=color_rmse, linewidth=1.6, label="rmse")
    if stats_dict.get("median") is not None:
        ax_top.axhline(stats_dict["median"], color=color_median, linewidth=1.6, label="median")
    if stats_dict.get("mean") is not None:
        ax_top.axhline(stats_dict["mean"], color=color_mean, linewidth=1.6, label="mean")
    if stats_dict.get("std") is not None:
        ax_top.axhline(stats_dict["std"], color=color_std, linewidth=1.6, label="std")

    align_text = "with SE(3) Umeyama alignment" if align_used else "not aligned"
    ax_top.set_title(f"{metric_name} w.r.t. translation part (m)\n({align_text})", fontsize=11)
    ax_top.set_xlabel("t (s)")
    ax_top.set_ylabel(f"{metric_name} (m)")
    ax_top.legend(loc="upper left", fontsize=8, frameon=True)

    ax_tbl.axis("off")
    stats_cm = [
        ("Prosječna pogreška", stats_dict.get("mean")),
        ("Medijalna pogreška", stats_dict.get("median")),
        ("Maksimalna pogreška", stats_dict.get("max")),
        ("Minimalna pogreška", stats_dict.get("min")),
        ("Standardna devijacija", stats_dict.get("std")),
    ]

    table_data = [["Mjerna jedinica - cm", "Vrijednost"]]
    for label, value_m in stats_cm:
        val_cm = "" if value_m is None else f"{value_m * 100.0:.2f}"
        table_data.append([label, val_cm])

    tbl = ax_tbl.table(
        cellText=table_data,
        loc="center",
        cellLoc="center",
        colLoc="center",
        edges="horizontal",
    )
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(10)
    tbl.scale(1.15, 1.7)

    for (r, c), cell in tbl.get_celld().items():
        if r == 0:
            cell.set_text_props(weight="bold")
            cell.set_facecolor("#f0f0f0")
        else:
            if c == 0:
                cell.set_text_props(weight="bold")
        cell.set_edgecolor("#444444")

    points = np.column_stack([xs, ys]).reshape(-1, 1, 2)
    if len(points) >= 2:
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        seg_vals = err[:-1]
        norm = plt.Normalize(vmin=float(np.nanmin(err)), vmax=float(np.nanmax(err)) if np.nanmax(err) > np.nanmin(err) else float(np.nanmin(err) + 1e-12))
        lc = LineCollection(segments, cmap="jet", norm=norm)
        lc.set_array(seg_vals)
        lc.set_linewidth(2.2)
        ax_traj.add_collection(lc)
        cbar = fig.colorbar(lc, ax=ax_traj, fraction=0.046, pad=0.03)
        cbar.set_label(f"{metric_name.lower()} (m)")
    else:
        ax_traj.scatter(xs, ys, c=err, cmap="jet", s=20)
        norm = None

    ax_traj.plot(xs, ys, linestyle="--", color="gray", alpha=0.35, linewidth=1.0, label="reference")
    ax_traj.set_title(f"{metric_name} w.r.t. translation part (m)\n({align_text})", fontsize=12)
    ax_traj.set_xlabel("x (m)")
    ax_traj.set_ylabel("y (m)")
    ax_traj.axis("equal")
    ax_traj.grid(True, alpha=0.3)

    pad_x = max((np.max(xs) - np.min(xs)) * 0.05, 0.1)
    pad_y = max((np.max(ys) - np.min(ys)) * 0.05, 0.1)
    ax_traj.set_xlim(np.min(xs) - pad_x, np.max(xs) + pad_x)
    ax_traj.set_ylim(np.min(ys) - pad_y, np.max(ys) + pad_y)

    fig.text(0.26, 0.495, f"Prikaz dobivenih vrijednosti iz evo paketa ({metric_name})", ha="center", fontsize=12)

    plt.savefig(out_png, dpi=180, bbox_inches="tight")
    plt.close(fig)
    return True


class NavEvoPipeline(Node):
    def __init__(self, args):
        super().__init__("nav_evo_pipeline")
        self.input_dir = Path(args.input_dir).expanduser().resolve()
        self.rpe_delta_m = float(args.rpe_delta_m)
        self.use_align = bool(args.evo_align)

        self.evo_ape_bin = shutil.which("evo_ape")
        self.evo_rpe_bin = shutil.which("evo_rpe")

    def run_cmd(self, cmd, out_txt: Path):
        proc = subprocess.run(cmd, capture_output=True, text=True)
        with open(out_txt, "w") as f:
            f.write("$ " + " ".join(cmd) + "\n\n")
            if proc.stdout:
                f.write("STDOUT\n")
                f.write(proc.stdout)
                f.write("\n")
            if proc.stderr:
                f.write("\nSTDERR\n")
                f.write(proc.stderr)
                f.write("\n")
        return proc.returncode, proc.stdout, proc.stderr

    def process_run(self, run_dir: Path):
        plan_csv = run_dir / "plan.csv"
        odom_csv = run_dir / "odom.csv"
        metadata_path = run_dir / "metadata.json"

        plan_rows = load_plan(plan_csv)
        odom_rows = load_odom(odom_csv)

        if not plan_rows:
            self.get_logger().warning(f"{run_dir}: plan.csv je prazan, preskačem.")
            return None
        if not odom_rows:
            self.get_logger().warning(f"{run_dir}: odom.csv je prazan, preskačem.")
            return None

        with open(metadata_path) as f:
            md = json.load(f)

        plan_time_rows = build_plan_time(plan_rows, odom_rows)

        plan_time_csv = run_dir / "plan_time.csv"
        plan_time_tum = run_dir / "plan_time.tum"
        odom_tum = run_dir / "odom.tum"

        write_csv(plan_time_csv, ["t", "x", "y", "yaw"], plan_time_rows)
        write_tum(plan_time_tum, plan_time_rows)
        write_tum(odom_tum, odom_rows)

        self.get_logger().info(
            f"{run_dir.name}: napravljeni plan_time.csv, plan_time.tum i odom.tum"
        )

        result = {
            "run_name": md.get("run_name"),
            "planner": md.get("planner"),
            "controller": md.get("controller"),
            "scenario": md.get("scenario"),
            "success": int(bool(md.get("success"))),
            "result_status": md.get("result_status"),
            "planning_time_s": md.get("planning_time_s"),
            "navigation_time_s": md.get("navigation_time_s"),
            "min_obstacle_distance_m": md.get("min_obstacle_distance_m"),
            "odom_samples": md.get("odom_samples"),
            "ape_rmse": None,
            "ape_mean": None,
            "ape_median": None,
            "ape_std": None,
            "ape_min": None,
            "ape_max": None,
            "rpe_rmse": None,
            "rpe_mean": None,
            "rpe_median": None,
            "rpe_std": None,
            "rpe_min": None,
            "rpe_max": None,
        }

        if not self.evo_ape_bin or not self.evo_rpe_bin:
            self.get_logger().warning(
                f"{run_dir.name}: evo nije pronađen u PATH-u, preskačem evo_ape/evo_rpe."
            )
            return result

        evo_dir = run_dir / "evo"
        evo_dir.mkdir(parents=True, exist_ok=True)

        ape_zip = evo_dir / "ape.zip"
        rpe_zip = evo_dir / "rpe.zip"
        ape_txt = evo_dir / "ape.txt"
        rpe_txt = evo_dir / "rpe.txt"
        ape_png = evo_dir / "ape_report.png"
        rpe_png = evo_dir / "rpe_report.png"

        ape_cmd = [
            self.evo_ape_bin,
            "tum",
            str(plan_time_tum),
            str(odom_tum),
            "-r", "trans_part",
            "--save_results", str(ape_zip),
        ]
        if self.use_align:
            ape_cmd.append("--align")

        rpe_cmd = [
            self.evo_rpe_bin,
            "tum",
            str(plan_time_tum),
            str(odom_tum),
            "-r", "trans_part",
            "--delta", str(self.rpe_delta_m),
            "--delta_unit", "m",
            "--all_pairs",
            "--save_results", str(rpe_zip),
        ]
        if self.use_align:
            rpe_cmd.append("--align")

        ape_rc, ape_stdout, ape_stderr = self.run_cmd(ape_cmd, ape_txt)
        rpe_rc, rpe_stdout, rpe_stderr = self.run_cmd(rpe_cmd, rpe_txt)

        if ape_rc != 0:
            self.get_logger().error(f"{run_dir.name}: evo_ape nije uspio, vidi {ape_txt}")
        else:
            ape_stats = parse_evo_stats(ape_stdout + "\n" + ape_stderr)
            result["ape_rmse"] = ape_stats["rmse"]
            result["ape_mean"] = ape_stats["mean"]
            result["ape_median"] = ape_stats["median"]
            result["ape_std"] = ape_stats["std"]
            result["ape_min"] = ape_stats["min"]
            result["ape_max"] = ape_stats["max"]
            self.get_logger().info(f"{run_dir.name}: evo_ape gotov")

            try:
                ok = generate_metric_report_png(
                    zip_path=ape_zip,
                    metric_name="APE",
                    out_png=ape_png,
                    tum_fallback=odom_tum,
                    stats_dict=ape_stats,
                    align_used=self.use_align,
                )
                if ok:
                    self.get_logger().info(f"{run_dir.name}: spremljen {ape_png.name}")
                else:
                    self.get_logger().warning(f"{run_dir.name}: nije moguće generirati APE report PNG.")
            except Exception as exc:
                self.get_logger().error(f"{run_dir.name}: greška pri izradi APE slike: {exc}")

        if rpe_rc != 0:
            self.get_logger().error(f"{run_dir.name}: evo_rpe nije uspio, vidi {rpe_txt}")
        else:
            rpe_stats = parse_evo_stats(rpe_stdout + "\n" + rpe_stderr)
            result["rpe_rmse"] = rpe_stats["rmse"]
            result["rpe_mean"] = rpe_stats["mean"]
            result["rpe_median"] = rpe_stats["median"]
            result["rpe_std"] = rpe_stats["std"]
            result["rpe_min"] = rpe_stats["min"]
            result["rpe_max"] = rpe_stats["max"]
            self.get_logger().info(f"{run_dir.name}: evo_rpe gotov")

            try:
                ok = generate_metric_report_png(
                    zip_path=rpe_zip,
                    metric_name="RPE",
                    out_png=rpe_png,
                    tum_fallback=odom_tum,
                    stats_dict=rpe_stats,
                    align_used=self.use_align,
                )
                if ok:
                    self.get_logger().info(f"{run_dir.name}: spremljen {rpe_png.name}")
                else:
                    self.get_logger().warning(f"{run_dir.name}: nije moguće generirati RPE report PNG.")
            except Exception as exc:
                self.get_logger().error(f"{run_dir.name}: greška pri izradi RPE slike: {exc}")

        return result

    def write_runs_summary(self, runs, out_csv: Path):
        fieldnames = [
            "run_name", "planner", "controller", "scenario",
            "success", "result_status",
            "planning_time_s", "navigation_time_s",
            "min_obstacle_distance_m", "odom_samples",
            "ape_rmse", "ape_mean", "ape_median", "ape_std", "ape_min", "ape_max",
            "rpe_rmse", "rpe_mean", "rpe_median", "rpe_std", "rpe_min", "rpe_max",
        ]
        with open(out_csv, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for r in runs:
                writer.writerow(r)

    def write_combo_summary(self, runs, out_csv: Path):
        grouped = defaultdict(list)
        for r in runs:
            grouped[(r["planner"], r["controller"])].append(r)

        fieldnames = [
            "planner", "controller", "runs", "success_rate",
            "planning_time_mean_s", "navigation_time_mean_s",
            "min_obstacle_distance_mean_m",
            "ape_rmse_mean", "ape_rmse_std",
            "ape_mean_mean", "ape_mean_std",
            "ape_max_mean",
            "rpe_rmse_mean", "rpe_rmse_std",
            "rpe_mean_mean", "rpe_mean_std",
            "rpe_max_mean",
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
                    "min_obstacle_distance_mean_m": mean(r["min_obstacle_distance_m"] for r in items),
                    "ape_rmse_mean": mean(r["ape_rmse"] for r in items),
                    "ape_rmse_std": stddev(r["ape_rmse"] for r in items),
                    "ape_mean_mean": mean(r["ape_mean"] for r in items),
                    "ape_mean_std": stddev(r["ape_mean"] for r in items),
                    "ape_max_mean": mean(r["ape_max"] for r in items),
                    "rpe_rmse_mean": mean(r["rpe_rmse"] for r in items),
                    "rpe_rmse_std": stddev(r["rpe_rmse"] for r in items),
                    "rpe_mean_mean": mean(r["rpe_mean"] for r in items),
                    "rpe_mean_std": stddev(r["rpe_mean"] for r in items),
                    "rpe_max_mean": mean(r["rpe_max"] for r in items),
                })

    def run(self):
        if not self.input_dir.exists():
            raise FileNotFoundError(f"Input direktorij ne postoji: {self.input_dir}")

        run_dirs = find_run_dirs(self.input_dir)
        self.get_logger().info(f"Pronađeno run direktorija: {len(run_dirs)}")

        runs = []
        for run_dir in run_dirs:
            try:
                result = self.process_run(run_dir)
                if result is not None:
                    runs.append(result)
            except Exception as exc:
                self.get_logger().error(f"{run_dir}: greška: {exc}")

        runs_summary_path = self.input_dir / "evo_runs_summary.csv"
        combo_summary_path = self.input_dir / "evo_combo_summary.csv"

        self.write_runs_summary(runs, runs_summary_path)
        self.write_combo_summary(runs, combo_summary_path)

        self.get_logger().info(f"Wrote: {runs_summary_path}")
        self.get_logger().info(f"Wrote: {combo_summary_path}")
        self.get_logger().info(f"Gotovo. Obrađeno runova: {len(runs)}")


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Za svaki run u --input_dir generira plan_time.csv, .tum datoteke, "
            "pokreće evo_ape/evo_rpe, sprema per-run PNG report slike "
            "i piše evo_runs_summary.csv i evo_combo_summary.csv."
        )
    )
    parser.add_argument(
        "--input_dir",
        required=True,
        help="Root direktorij koji sadrži run direktorije."
    )
    parser.add_argument(
        "--rpe_delta_m",
        type=float,
        default=0.5,
        help="Delta u metrima za evo_rpe (default: 0.5)."
    )
    parser.add_argument(
        "--evo_align",
        action="store_true",
        help="Ako je zadano, koristi --align za evo_ape i evo_rpe."
    )

    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = NavEvoPipeline(args)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
