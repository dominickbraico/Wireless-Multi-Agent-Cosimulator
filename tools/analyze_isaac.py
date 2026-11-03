#!/usr/bin/env python3
"""Run-validity report for an Isaac Sim run (simulator.type isaac): ns-3 lag and the simulator's real-time factor.

    analyze_isaac.py <log dir> <experiment_run.yaml>

Isaac Sim has no stats topic like Gazebo's, so the factor comes from the pose feed (tools/timehealth.py rtf_from_poses: simulated seconds over wall seconds
across 5 applied samples of the first agent that has samples). Two windows, both against time.min_rtf with the sliding mean of time.rtf_window_s:
  flight window  from the first sample whose antenna z is 0.3 m above the median z of the first 10 s of the stream, to the last sample. This is the criterion.
  whole stream   from 5 s after the first sample to the last sample. Reported for information (it includes start-up and lockstep waits for PX4).
A run with no pose samples, or with no flight in it, is reported as "run not certified: <reason>" and exits 1; the number is never guessed.
Also checks ns-3 real-time lag against time.max_lag_ms from ns3/summary.json.
"""
import csv
import json
import os
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from timehealth import rtf_check, rtf_from_poses

log, cfg_f = sys.argv[1:3]
cfg = yaml.safe_load(open(cfg_f))
rmin, win, lag_max = cfg["time"]["min_rtf"], cfg["time"].get("rtf_window_s", 5.0), cfg["time"]["max_lag_ms"]
ok_all = True


def check(name, cond, detail=""):
    global ok_all
    print(("PASS  " if cond else "FAIL  ") + name + (f"  ({detail})" if detail else ""))
    ok_all &= bool(cond)


try:
    poses = list(csv.DictReader(open(f"{log}/ns3/pose_applied.csv")))
except FileNotFoundError:
    poses = []

node = next((i for i in range(len(cfg["agents"])) if sum(1 for r in poses if int(r["node"]) == i) > 5), None)
if node is None:
    print("run not certified: no pose samples from any agent in ns3/pose_applied.csv, so there is no real-time factor for Isaac Sim")
    sys.exit(1)

samples = rtf_from_poses(poses, node)
mine = sorted((float(r["t_wall_apply_s"]), float(r["z"])) for r in poses if int(r["node"]) == node)
t_first, t_last = mine[0][0], mine[-1][0]
whole = rtf_check(samples, t_first + win, t_last, rmin, win)
if whole["n"]:
    print(f"RTF whole stream (information, agent index {node}): worst {win:g}-s mean {whole['worst_window_mean']:.3f}, worst sample {whole['worst_sample']:.2f}, "
          f"{whole['n']} samples, min_rtf {rmin}")

base = sorted(z for t, z in mine if t <= t_first + 10.0)
z0 = base[len(base) // 2]
lift = next((t for t, z in mine if z > z0 + 0.3), None)
if lift is None:
    print(f"run not certified: the vehicle never rose 0.3 m above its resting height ({z0:.2f} m), so there is no flight window to judge")
    ok_all = False
else:
    fl = rtf_check(samples, lift, t_last, rmin, win)
    check(f"RTF flight window {win:g}-s sliding mean >= {rmin}", fl["ok"],
          f"worst window mean {fl['worst_window_mean']:.3f}, worst sample {fl['worst_sample']:.2f}, {fl['n']} samples, flight {t_last - lift:.1f} s" if fl["n"] else "no samples")

try:
    summ = json.load(open(f"{log}/ns3/summary.json"))
    check("ns-3 real-time lag within limit", summ["valid_ns3_lag"], f"max {summ['ns3_max_lag_ms']} ms, limit {lag_max} ms")
except FileNotFoundError:
    check("ns-3 summary.json written (clean shutdown)", False)
sys.exit(0 if ok_all else 1)
