#!/usr/bin/env python3
"""Pose-feed analysis: compare ns-3's applied positions with what Gazebo was told.

usage: check_pose_feed.py <ns3_log_dir> <bridge_log_dir> <steps.json> <config.yaml>
steps.json (from adapters/gazebo/sweep.py): commanded model positions + wall end times.
Checks per step: the position ns-3 held just before the step ended equals the
commanded position + antenna offset (1 mm). Globally: staleness, malformed
datagrams, ns-3 lag, and Gazebo real-time factor over the measured window.
Exit 0 on pass.
"""
import csv
import json
import sys

import yaml

ns3_dir, bridge_dir, steps_f, cfg_f = sys.argv[1:5]
cfg = yaml.safe_load(open(cfg_f))
names = [a["name"] for a in cfg["agents"]]
off = {a["name"]: a["antenna_offset"] for a in cfg["agents"]}
period_ms = 1000.0 / cfg["network"]["pose"]["rate_hz"]
min_rtf = cfg["time"]["min_rtf"]
sw = json.load(open(steps_f))
summ = json.load(open(f"{ns3_dir}/summary.json"))
rows = list(csv.DictReader(open(f"{ns3_dir}/pose_applied.csv")))
ok = True


def check(name, cond, detail=""):
    global ok
    print(("PASS  " if cond else "FAIL  ") + name + (f"  ({detail})" if detail else ""))
    ok &= bool(cond)


by_node = {}
for r in rows:
    by_node.setdefault(int(r["node"]), []).append(
        (float(r["t_wall_apply_s"]), [float(r["x"]), float(r["y"]), float(r["z"])]))

for s in sw["steps"]:
    n = names.index(s["agent"])
    want = [s["pos"][i] + off[s["agent"]][i] for i in range(3)]
    hist = [p for t, p in by_node.get(n, []) if t <= s["t_wall_end"]]
    err = max(abs(hist[-1][i] - want[i]) for i in range(3)) if hist else 1e9
    check(f"{s['agent']} step to {s['pos']}: ns-3 holds commanded+offset", err < 1e-3, f"err {err*1000:.3f} mm")

n_dg, n_app = summ["pose_datagrams"], summ["pose_applied"]
check("no malformed pose datagrams", summ["pose_bad_datagrams"] == 0, str(summ["pose_bad_datagrams"]))
check("every agent entry in every datagram applied", n_app == n_dg * len(names),
      f"{n_app} applied / {n_dg} datagrams x {len(names)}")
check(f"max pose staleness < one update period ({period_ms:.0f} ms)", summ["pose_max_stale_ms"] < period_ms,
      f"{summ['pose_max_stale_ms']:.2f} ms")
check("ns-3 real-time lag within limit", summ["valid_ns3_lag"], f"max {summ['ns3_max_lag_ms']} ms")

sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
from timehealth import rtf_check
rtf = [(float(r["t_wall_s"]), float(r["rtf"])) for r in csv.DictReader(open(f"{bridge_dir}/rtf.csv"))]
rc = rtf_check(rtf, sw["t_window_start"], sw["t_window_end"], min_rtf, cfg["time"].get("rtf_window_s", 5.0))
check(f"Gazebo RTF sliding mean >= {min_rtf} over measured window", rc["ok"],
      f"worst window mean {rc['worst_window_mean']:.3f}, worst sample {rc['worst_sample']:.2f}" if rc["n"] else "no samples")
sys.exit(0 if ok else 1)
