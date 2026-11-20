#!/usr/bin/env python3
"""Evaluate an exploration run against the true floor plan.

Once per second of simulated time it records the robot's true position (ground_truth/odometry), the distance driven,
and how much of the floor the SLAM map (map) has covered: the fraction of the free cells reachable from the start
that the map knows to be free. When the explorer publishes exploration/finished (or after `timeout`), it also
measures the map's accuracy, writes a CSV, prints a summary and exits.

The truth comes from the wall list that lidar_exploration_gazebo installs (worlds/office_floor_walls.json). The map
frame is the odometry frame at the start, so a map point p is at R(yaw0) p + (x0, y0) in the world.

Parameters: output (CSV path, default exploration_run.csv), walls_file (default: the installed JSON),
timeout (s of simulated time, default 1500), settle_time (s to wait for the last map, default 3.0).
"""
import csv
import json
import math
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty

RESOLUTION = 0.05  # truth grid [m]


class Truth:
    """Ground-truth grid of the floor at RESOLUTION, in the world frame."""

    def __init__(self, walls_file):
        spec = json.loads(Path(walls_file).read_text())
        self.start = spec["start"]
        walls = np.array(spec["walls"], dtype=float)
        half = 0.5 * spec["thickness"]
        ends = np.vstack([walls[:, :2], walls[:, 2:]])
        lo, hi = ends.min(0) - 0.5, ends.max(0) + 0.5
        self.origin = lo
        nx, ny = np.ceil((hi - lo) / RESOLUTION).astype(int)
        xs = lo[0] + (np.arange(nx) + 0.5) * RESOLUTION
        ys = lo[1] + (np.arange(ny) + 0.5) * RESOLUTION
        self.centres = np.stack(np.meshgrid(xs, ys), axis=-1)  # [ny, nx, 2]
        dist = np.full((ny, nx), np.inf)
        for ax, ay, bx, by in walls:
            a, d = np.array([ax, ay]), np.array([bx - ax, by - ay])
            u = np.clip(((self.centres - a) @ d) / (d @ d), 0.0, 1.0)
            dist = np.minimum(dist, np.linalg.norm(self.centres - a - u[..., None] * d, axis=-1))
        self.wall_distance = dist  # distance from each cell centre to the nearest wall centre line
        self.occupied = dist <= half + 0.5 * RESOLUTION
        self.reachable = self._flood(~self.occupied, self.cell(self.start[0], self.start[1]))

    def cell(self, x, y):
        return int((y - self.origin[1]) / RESOLUTION), int((x - self.origin[0]) / RESOLUTION)

    @staticmethod
    def _flood(free, seed):
        seen = np.zeros_like(free)
        seen[seed] = True
        queue = deque([seed])
        ny, nx = free.shape
        while queue:
            r, c = queue.popleft()
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                rr, cc = r + dr, c + dc
                if 0 <= rr < ny and 0 <= cc < nx and free[rr, cc] and not seen[rr, cc]:
                    seen[rr, cc] = True
                    queue.append((rr, cc))
        return seen


class Evaluator(Node):
    def __init__(self):
        super().__init__("exploration_evaluator")
        default_walls = Path(get_package_share_directory("lidar_exploration_gazebo")) / "worlds" / \
            "office_floor_walls.json"
        self.output = self.declare_parameter("output", "exploration_run.csv").value
        self.timeout = self.declare_parameter("timeout", 1500.0).value
        self.settle = self.declare_parameter("settle_time", 3.0).value
        self.truth = Truth(self.declare_parameter("walls_file", str(default_walls)).value)
        x0, y0, yaw0 = self.truth.start
        c, s = math.cos(yaw0), math.sin(yaw0)
        # World -> map for the truth cell centres, to look them up in the SLAM map.
        p = self.truth.centres - np.array([x0, y0])
        self.centres_in_map = np.stack([c * p[..., 0] + s * p[..., 1], -s * p[..., 0] + c * p[..., 1]], axis=-1)
        self.map = None
        self.pose = None
        self.distance = 0.0
        self.rows = []
        self.t0 = None
        self.finished_at = None
        self.create_subscription(OccupancyGrid, "map", self.on_map,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Odometry, "ground_truth/odometry", self.on_truth, 50)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Empty, "exploration/finished", self.on_finished, latched)
        self.create_timer(1.0, self.sample)
        reach = self.truth.reachable.sum() * RESOLUTION ** 2
        self.get_logger().info(f"truth: {reach:.1f} m^2 of free floor reachable from the start")

    def on_map(self, m):
        self.map = m

    def on_truth(self, m):
        p = (m.pose.pose.position.x, m.pose.pose.position.y)
        if self.pose is not None:
            self.distance += math.hypot(p[0] - self.pose[0], p[1] - self.pose[1])
        self.pose = p

    def on_finished(self, _):
        if self.finished_at is None:
            self.finished_at = self.now()
            self.get_logger().info(f"explorer finished; evaluating in {self.settle:.1f} s")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def map_lookup(self, points):
        """Values of the SLAM map at map-frame points [..., 2]; -1 outside the map."""
        info = self.map.info
        grid = np.array(self.map.data, dtype=np.int16).reshape(info.height, info.width)
        col = np.floor((points[..., 0] - info.origin.position.x) / info.resolution).astype(int)
        row = np.floor((points[..., 1] - info.origin.position.y) / info.resolution).astype(int)
        inside = (col >= 0) & (row >= 0) & (col < info.width) & (row < info.height)
        out = np.full(points.shape[:-1], -1, dtype=np.int16)
        out[inside] = grid[row[inside], col[inside]]
        return out

    def coverage(self):
        if self.map is None:
            return 0.0
        v = self.map_lookup(self.centres_in_map)
        known_free = (v >= 0) & (v < 50)
        return float((known_free & self.truth.reachable).sum() / self.truth.reachable.sum())

    def accuracy(self):
        """Occupied map cells within 0.15 m of a true wall surface, and free map cells whose centre is not inside a wall
        by more than half a cell (cells on the wall surface are partly free and partly wall)."""
        info = self.map.info
        grid = np.array(self.map.data, dtype=np.int16).reshape(info.height, info.width)
        rows, cols = np.nonzero(grid >= 0)
        x0, y0, yaw0 = self.truth.start
        c, s = math.cos(yaw0), math.sin(yaw0)
        mx = info.origin.position.x + (cols + 0.5) * info.resolution
        my = info.origin.position.y + (rows + 0.5) * info.resolution
        wx, wy = x0 + c * mx - s * my, y0 + s * mx + c * my
        r = np.floor((wy - self.truth.origin[1]) / RESOLUTION).astype(int)
        k = np.floor((wx - self.truth.origin[0]) / RESOLUTION).astype(int)
        ny, nx = self.truth.occupied.shape
        ok = (r >= 0) & (k >= 0) & (r < ny) & (k < nx)
        occ = grid[rows, cols] >= 50
        dist = np.full(rows.shape, np.inf)
        dist[ok] = self.truth.wall_distance[r[ok], k[ok]]
        occ_ok = (dist[occ] <= 0.05 + 0.15).mean() if occ.any() else 1.0
        free_ok = (dist[~occ] >= 0.05 - 0.5 * info.resolution).mean() if (~occ).any() else 1.0
        return float(occ_ok), float(free_ok)

    def sample(self):
        t = self.now()
        if t <= 0.0 or self.pose is None:
            return
        if self.t0 is None:
            self.t0 = t
        self.rows.append((t - self.t0, self.pose[0], self.pose[1], self.distance, self.coverage()))
        done = self.finished_at is not None and t - self.finished_at >= self.settle
        if done or t - self.t0 > self.timeout:
            self.report(finished=done)

    def report(self, finished):
        with open(self.output, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "x", "y", "distance", "coverage"])
            for r in self.rows:
                w.writerow([f"{v:.4f}" for v in r])
        t, _, _, dist, cov = self.rows[-1]
        occ_ok, free_ok = self.accuracy() if self.map is not None else (float("nan"), float("nan"))
        t90 = next((r[0] for r in self.rows if r[4] >= 0.90), float("nan"))
        explored = self.truth.reachable.sum() * RESOLUTION ** 2 * cov
        lines = [
            f"finished by the explorer   {'yes' if finished else 'NO (timeout)'}",
            f"duration                   {t:.0f} s (90 % coverage after {t90:.0f} s)",
            f"distance driven            {dist:.1f} m",
            f"coverage of reachable free {100 * cov:.1f} % ({explored:.1f} m^2)",
            f"map accuracy               occupied cells on a wall {100 * occ_ok:.1f} %, "
            f"free cells really free {100 * free_ok:.2f} %",
            f"written                    {self.output}",
        ]
        print("\n".join(lines), flush=True)
        raise SystemExit(0)


def main():
    rclpy.init()
    node = Evaluator()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
