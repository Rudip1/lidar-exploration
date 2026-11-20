#!/usr/bin/env python3
"""Plot exploration runs written by evaluate_exploration.py and print their summary numbers.

    python3 tools/plot_run.py docs/results/office_floor.csv docs/results/office_floor.png [more.csv ...]

Left: the robot's true path over the floor plan (first run). Right: coverage of the reachable free floor over time,
one curve per run.
"""
import csv
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

WALLS = Path(__file__).resolve().parent.parent / "lidar_exploration_gazebo" / "worlds" / "office_floor_walls.json"


def load(path):
    with open(path) as f:
        rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]
    return {k: [r[k] for r in rows] for k in rows[0]}


def summary(name, d):
    t90 = next((t for t, c in zip(d["t"], d["coverage"]) if c >= 0.9), float("nan"))
    print(f"{name}: {d['t'][-1]:.0f} s, {d['distance'][-1]:.1f} m driven, coverage {100 * d['coverage'][-1]:.1f} %, "
          f"90 % after {t90:.0f} s")


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    *runs, dst = sys.argv[1:]
    data = [load(r) for r in runs]
    for r, d in zip(runs, data):
        summary(Path(r).stem, d)

    spec = json.loads(WALLS.read_text())
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(14, 5.2), gridspec_kw={"width_ratios": [1.45, 1]})
    for ax0, ay0, ax1, ay1 in spec["walls"]:
        ax.plot([ax0, ax1], [ay0, ay1], color="0.45", lw=3, solid_capstyle="butt")
    d = data[0]
    ax.plot(d["x"], d["y"], color="tab:blue", lw=1.3, label="robot (ground truth)")
    ax.plot(d["x"][0], d["y"][0], "o", color="tab:green", ms=8, label="start")
    ax.plot(d["x"][-1], d["y"][-1], "s", color="tab:red", ms=7, label="end")
    ax.set_aspect("equal")
    ax.set_title(f"path in the office floor (Gazebo, {Path(runs[0]).stem})")
    ax.legend(loc="upper right", fontsize=8, framealpha=0.9)
    ax.grid(alpha=0.3)

    for r, d in zip(runs, data):
        bx.plot(d["t"], [100 * c for c in d["coverage"]], lw=1.5, label=Path(r).stem)
    bx.axhline(90, color="0.5", ls=":", lw=1)
    bx.set_xlabel("time [s]")
    bx.set_ylabel("reachable free floor mapped [%]")
    bx.set_ylim(0, 102)
    bx.set_title("coverage")
    bx.grid(alpha=0.3)
    bx.legend(fontsize=8, loc="lower right")
    fig.tight_layout()
    fig.savefig(dst, dpi=110)
    print(f"wrote {dst}")


if __name__ == "__main__":
    main()
