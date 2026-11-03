#!/usr/bin/env python3
"""Pass/fail analysis of a container run of the rover-drive scenario (scenario.type rover_drive; docs/EXPERIMENTS.md, "Rover drive test").

    analyze_rover.py <log dir> <experiment_run.yaml>

For every agent it checks, in this order (thresholds fixed in the docs before the first run, not tuned afterwards):
  1. the drive node armed in offboard mode, reached all four corners and disarmed (from <agent>_flight.log);
  2. PX4's own position (rover/rover_<agent>.csv) passed within ODOM_TOL_M of every corner;
  3. the simulator's ground truth, as applied in ns-3 (ns3/pose_applied.csv, ENU metres, Gazebo world frame), passed within TRUTH_TOL_M
     of every corner and ended within END_TOL_M of the spawn point (this is what shows the vehicle moved in Gazebo, not just in PX4's estimate);
  4. the vehicle stayed on the ground and upright enough not to flip or jump: ground-truth z varied by at most Z_RANGE_M while driving.
Run validity (same checks as tools/analyze_formation.py): ns-3 real-time lag within time.max_lag_ms and Gazebo RTF sliding mean >= time.min_rtf
between the go signal and the last pose.

Corner geometry: PX4 local frame is NED with its origin at the arming point; ENU world x = East, y = North, so the corners North s, East s
become world offsets (0, s), (s, s), (s, 0), (0, 0) from the spawn point. Assumes the PX4 local frame is aligned with the world axes (spawn yaw 0).
"""
import csv
import json
import math
import os
import sys

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "orchestrator"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scenario_rover
from timehealth import rtf_check

ODOM_TOL_M, TRUTH_TOL_M, END_TOL_M, Z_RANGE_M = 0.75, 1.0, 1.5, 0.35

log, cfg_f = sys.argv[1:3]
cfg = yaml.safe_load(open(cfg_f))
side = float(scenario_rover.settings(cfg)["side_m"])
ok_all = True


def check(name, cond, detail=""):
    global ok_all
    print(("PASS  " if cond else "FAIL  ") + name + (f"  ({detail})" if detail else ""))
    ok_all &= bool(cond)


def text(path):
    try:
        return open(path, errors="ignore").read()
    except FileNotFoundError:
        return ""


def rows(path):
    try:
        return list(csv.DictReader(open(path)))
    except FileNotFoundError:
        return []


go = f"{log}/go"
t_go = os.path.getmtime(go) if os.path.exists(go) else 0.0
poses = rows(f"{log}/ns3/pose_applied.csv")
report = {"side_m": side, "agents": {}}

for idx, a in enumerate(cfg["agents"]):
    name = a["name"]
    sx, sy = float(a["spawn_pose"][0]), float(a["spawn_pose"][1])
    flight = text(f"{log}/{name}_flight.log")
    check(f"{name} armed in offboard mode", "Armed in offboard mode" in flight)
    check(f"{name} reached all 4 corners", "Reached corner 4 of 4" in flight, "" if "Reached corner 4 of 4" in flight else "see flight log")
    check(f"{name} route complete and disarmed", "Route complete and disarmed" in flight)

    odo = rows(f"{log}/rover/rover_{name}.csv")
    drive = [(float(r["x_north_m"]), float(r["y_east_m"])) for r in odo if r["state"] == "DRIVE"]
    corners_ne = [(side, 0.0), (side, side), (0.0, side), (0.0, 0.0)]       # North, East offsets from the arming point
    if drive:
        hx, hy = drive[0]
        worst = max(min(math.hypot(x - (hx + cn), y - (hy + ce)) for x, y in drive) for cn, ce in corners_ne)
        check(f"{name} PX4 position within {ODOM_TOL_M} m of every corner", worst <= ODOM_TOL_M, f"worst closest approach {worst:.2f} m")
    else:
        worst = None
        check(f"{name} PX4 position log has driving samples", False, "rover/rover_%s.csv missing or empty" % name)

    mine = [r for r in poses if int(r["node"]) == idx and float(r["t_wall_apply_s"]) >= t_go]
    xy = [(float(r["x"]), float(r["y"])) for r in mine]
    zs = [float(r["z"]) for r in mine]
    corners_w = [(sx + e, sy + n) for n, e in corners_ne]                    # ENU world
    if xy:
        tw = max(min(math.hypot(x - cx, y - cy) for x, y in xy) for cx, cy in corners_w)
        end = math.hypot(xy[-1][0] - sx, xy[-1][1] - sy)
        zr = max(zs) - min(zs)
        check(f"{name} Gazebo ground truth within {TRUTH_TOL_M} m of every corner", tw <= TRUTH_TOL_M, f"worst closest approach {tw:.2f} m")
        check(f"{name} ended within {END_TOL_M} m of its spawn point (ground truth)", end <= END_TOL_M, f"{end:.2f} m")
        check(f"{name} ground-truth z varied by at most {Z_RANGE_M} m", zr <= Z_RANGE_M, f"{zr:.2f} m")
        report["agents"][name] = {"odom_worst_m": worst, "truth_worst_m": tw, "end_m": end, "z_range_m": zr, "pose_samples": len(xy)}
    else:
        check(f"{name} ground-truth poses present after the go signal", False, "ns3/pose_applied.csv has none for this agent")

# ---- run validity ----
try:
    summ = json.load(open(f"{log}/ns3/summary.json"))
    check("ns-3 real-time lag within limit", summ["valid_ns3_lag"], f"max {summ['ns3_max_lag_ms']} ms")
except FileNotFoundError:
    check("ns-3 summary.json written (clean shutdown)", False)
rt = [(float(r["t_wall_s"]), float(r["rtf"])) for r in rows(f"{log}/bridge/rtf.csv")]
if rt and poses:
    win, rmin = cfg["time"].get("rtf_window_s", 5.0), cfg["time"]["min_rtf"]
    t_end = max(float(r["t_wall_apply_s"]) for r in poses)
    rc = rtf_check(rt, max(t_go, float(poses[0]["t_wall_apply_s"]) + 5), t_end, rmin, win)
    check(f"Gazebo RTF {win:g}-s sliding mean >= {rmin}", rc["ok"],
          f"worst window mean {rc['worst_window_mean']:.3f}, worst sample {rc['worst_sample']:.2f}" if rc["n"] else "no samples")
    report["rtf_worst_window_mean"] = rc["worst_window_mean"]
else:
    check("Gazebo RTF samples present", False)

json.dump(report, open(f"{log}/rover_report.json", "w"), indent=1)
print("RESULT: " + ("PASS" if ok_all else "FAIL"))
sys.exit(0 if ok_all else 1)
