#!/usr/bin/env python3
import csv
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def quat_to_yaw(x, y, z, w):
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z)
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


class NavBenchmarkRunner(Node):
    def __init__(self):
        super().__init__("nav_benchmark_runner")

        self.declare_parameter(
            "output_dir",
            str(Path.home() / "pas_seminar" / "metrics" / "metrics_sim")
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
        self.declare_parameter("scan_topic", "base_scan")
        self.declare_parameter("compute_path_action", "compute_path_to_pose")
        self.declare_parameter("navigate_action", "navigate_to_pose")

        self.output_dir = Path(self.get_parameter("output_dir").value)
        self.run_name = str(self.get_parameter("run_name").value)
        self.planner_id = str(self.get_parameter("planner_id").value)
        self.planner_label = str(self.get_parameter("planner_label").value)
        self.controller_label = str(self.get_parameter("controller_label").value)
        self.scenario = str(self.get_parameter("scenario").value)

        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.goal_yaw = float(self.get_parameter("goal_yaw").value)

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
            50
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_cb,
            50
        )

        self.compute_client = ActionClient(
            self,
            ComputePathToPose,
            compute_path_action
        )
        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            navigate_action
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
                min(finite_ranges)
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

    def save_metadata(self, planning_time_s, nav_time_s, result_status):
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

        self.get_logger().info("Sending NavigateToPose goal")
        result_status, nav_time_s = self.navigate()

        self.save_odom()
        self.save_metadata(planning_time_s, nav_time_s, result_status)

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
