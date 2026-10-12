#!/usr/bin/env python3
"""Leader-follower analysis: how well does the follower hold formation, and did ONLY swarm traffic use the radio?

  analyze_formation.py <log_dir> <experiment.yaml> [--offset x,y,z] [--max-p95 M]
One leader, any number of followers (orchestrator/fleet.py decides who is who and each follower's offset; --offset
forces one offset on every follower). Ground truth = ns-3's applied ghost positions (== Gazebo poses from the pose
bridge); ns-3 node i is the i-th agent of the YAML.
Formation error e(t) = |(p_follower - p_leader) - offset|, evaluated only while the leader is moving, per follower.
The acceptance threshold (--max-p95, default 5 m) was fixed BEFORE the first run; the measured numbers are reported as-is.
Also reports, per follower, swarm-link loss and latency against the mean distance to the leader.
Writes formation.png, link_vs_distance.png (more than one follower) and formation_report.json. Exit 0 if every check passes.
"""
import csv
import json
import math
import os
import subprocess
import sys

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "orchestrator"))
from fleet import resolve

log, cfg_f = sys.argv[1:3]
max_p95 = float(sys.argv[sys.argv.index("--max-p95") + 1]) if "--max-p95" in sys.argv else 5.0
off_arg = [float(v) for v in sys.argv[sys.argv.index("--offset") + 1].split(",")] if "--offset" in sys.argv else None
cfg = yaml.safe_load(open(cfg_f))
names = [a["name"] for a in cfg["agents"]]
leader, followers = resolve(cfg, off_arg)
LN = leader["name"]
ok_all = True


def check(name, cond, detail=""):
    global ok_all
    print(("PASS  " if cond else "FAIL  ") + name + (f"  ({detail})" if detail else ""))
    ok_all &= bool(cond)


def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(round(q * (len(v) - 1))))] if v else float("nan")


def text(path):
    try:
        return open(path, errors="ignore").read()
    except FileNotFoundError:
        return ""


# ---- flights ----
for a in [LN] + [f["name"] for f, _ in followers]:
    t = text(f"{log}/{a}_flight.log")
    check(f"{a} armed, took off and reached altitude", "Reached altitude" in t, "see flight log" if "Reached altitude" not in t else "")
check(f"leader {LN} finished its trajectory, landed and disarmed", "Landed and disarmed" in text(f"{log}/{LN}_flight.log"))
for f, _ in followers:
    check(f"follower {f['name']} landed after the leader link was lost", "Landed and disarmed" in text(f"{log}/{f['name']}_flight.log"),
          "follower on_link_lost=land")

# ---- ground truth: applied positions per ns-3 node (node i = i-th agent) ----
trk = {i: [] for i in range(len(names))}
for r in csv.DictReader(open(f"{log}/ns3/pose_applied.csv")):
    try:
        trk[int(r["node"])].append((float(r["t_wall_apply_s"]), float(r["x"]), float(r["y"]), float(r["z"])))
    except (ValueError, KeyError):
        pass
L = trk[names.index(LN)]
check("pose feed delivered every vehicle", all(len(v) > 100 for v in trk.values()), "/".join(str(len(v)) for v in trk.values()) + " samples")

# leader horizontal speed over 1 s
moving = []
j = 0
for i, (t, x, y, z) in enumerate(L):
    while L[j][0] < t - 1.0:
        j += 1
    dt = t - L[j][0]
    if dt > 0.5 and math.hypot(x - L[j][1], y - L[j][2]) / dt > 0.3 and z > 2.0:
        moving.append(t)
mv = (min(moving), max(moving)) if moving else None


def leader_cursor():
    idx = [0]
    def at(t):
        while idx[0] + 1 < len(L) and L[idx[0] + 1][0] <= t:
            idx[0] += 1
        return L[idx[0]]
    return at


report = {"followers": {}}
series_by, paths_by, dist_by = {}, {}, {}
for f, off in followers:
    n = f["name"]
    F = trk[names.index(n)]
    at = leader_cursor()
    errs, errz, series, dists = [], [], [], []
    for t, x, y, z in F:
        lt, lx, ly, lz = at(t)
        ex, ey, ez = x - lx - off[0], y - ly - off[1], z - lz - off[2]
        series.append((t, math.hypot(ex, ey)))
        if mv and mv[0] <= t <= mv[1]:
            errs.append(math.hypot(ex, ey))
            errz.append(abs(ez))
            dists.append(math.sqrt((x - lx) ** 2 + (y - ly) ** 2 + (z - lz) ** 2))
    r = {"offset_m": off}
    if errs:
        r.update({"moving_s": round(mv[1] - mv[0], 1), "n": len(errs), "horiz_err_p50_m": round(pct(errs, .5), 2),
                  "horiz_err_p95_m": round(pct(errs, .95), 2), "horiz_err_max_m": round(max(errs), 2),
                  "vert_err_p95_m": round(pct(errz, .95), 2), "mean_leader_distance_m": round(sum(dists) / len(dists), 1)})
        print(f"{n}: formation error while leader moves:", {k: v for k, v in r.items() if k != "offset_m"})
    check(f"{n} holds formation: horizontal error p95 <= {max_p95} m while the leader moves",
          bool(errs) and r["horiz_err_p95_m"] <= max_p95, str(r.get("horiz_err_p95_m")))

    # ---- swarm link as seen by this follower ----
    path = f"{log}/swarm_follower_{n}.csv"
    rows = list(csv.DictReader(open(path))) if os.path.exists(path) else []
    if rows:
        seqs = [int(x["seq"]) for x in rows]
        lat = [float(x["latency_ms"]) for x in rows]
        loss = 100 * (1 - len(rows) / (max(seqs) - min(seqs) + 1))
        r.update({"swarm_rx": len(rows), "swarm_loss_pct": round(loss, 2), "swarm_lat_p50_ms": round(pct(lat, .5), 2),
                  "swarm_lat_p99_ms": round(pct(lat, .99), 2)})
        print(f"{n}: swarm link:", {k: r[k] for k in ("swarm_rx", "swarm_loss_pct", "swarm_lat_p50_ms", "swarm_lat_p99_ms")})
    check(f"{n}: swarm stream received through ns-3 with < 5 % sequence loss", bool(rows) and r.get("swarm_loss_pct", 100) < 5)
    report["followers"][n] = r
    series_by[n], paths_by[n] = series, F

# ---- radio isolation: ONLY swarm (domain 42) traffic may be in the emulated channel ----
def count(pcap, flt, grep=None):
    cmd = ["tcpdump", "-Z", "root", "-r", pcap, "-nn"] + (["-A"] if grep else []) + [flt]
    out = subprocess.run(cmd, capture_output=True, text=True).stdout
    return sum(1 for l in out.splitlines() if grep in l) if grep else len([l for l in out.splitlines() if l.strip()])

pcap = f"{log}/ns3/wifi-{names.index(LN)}-0.pcap"
if os.path.exists(pcap):
    dom0 = count(pcap, "udp and portrange 7400-7650")
    xrce = count(pcap, "udp and port 8888")
    fmu = count(pcap, "udp", grep="fmu/")
    swarm = count(pcap, "udp and portrange 17900-18150")
    sw_topic = count(pcap, "udp", grep="swarm")
    check("no local-domain (0) RTPS ever on the emulated channel", dom0 == 0, f"{dom0} frames")
    check("no XRCE-DDS (port 8888) traffic on the emulated channel", xrce == 0, f"{xrce} frames")
    check("no PX4 topic names (fmu/...) anywhere on the emulated channel", fmu == 0, f"{fmu} frames")
    check("swarm domain (42) RTPS present on the emulated channel", swarm > 0 and sw_topic > 0, f"{swarm} frames, {sw_topic} mention 'swarm'")
    report.update({"pcap_domain0_frames": dom0, "pcap_xrce_frames": xrce, "pcap_fmu_frames": fmu, "pcap_swarm_frames": swarm})
else:
    check("ns-3 pcap present", False)

# ---- time health ----
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from timehealth import rtf_check
try:
    summ = json.load(open(f"{log}/ns3/summary.json"))
    check("ns-3 real-time lag within limit", summ["valid_ns3_lag"], f"max {summ['ns3_max_lag_ms']} ms")
except FileNotFoundError:
    check("ns-3 summary.json written (clean shutdown)", False)
rt = [(float(r["t_wall_s"]), float(r["rtf"])) for r in csv.DictReader(open(f"{log}/bridge/rtf.csv"))]
if rt and L:
    rc = rtf_check(rt, L[0][0] + 5, L[-1][0], cfg["time"]["min_rtf"], cfg["time"].get("rtf_window_s", 5.0))
    check(f"Gazebo RTF {cfg['time'].get('rtf_window_s', 5.0):g}-s sliding mean >= {cfg['time']['min_rtf']}", rc["ok"],
          f"worst window mean {rc['worst_window_mean']:.3f}, worst sample {rc['worst_sample']:.2f}" if rc["n"] else "no samples")
    report["rtf_worst_window_mean"] = rc["worst_window_mean"]

json.dump(report, open(f"{log}/formation_report.json", "w"), indent=1)

# ---- plots ----
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    SURF, INK, INK2, GRID, BLUE = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df", "#2a78d6"
    # one hue per follower; fixed order so a follower keeps its colour across plots
    PAL = ["#eb6834", "#2f9e6b", "#8a5cc2", "#d4a017", "#c2417a", "#4a90a4", "#7a7a2c", "#a0522d"]
    col = {f["name"]: PAL[i % len(PAL)] for i, (f, _) in enumerate(followers)}

    def style(axes):
        for a in axes:
            a.set_facecolor(SURF); a.grid(True, color=GRID, lw=0.8)
            for sp in ("top", "right"): a.spines[sp].set_visible(False)
            for sp in ("left", "bottom"): a.spines[sp].set_color(GRID)

    fig, ax = plt.subplots(1, 2, figsize=(13, 5), facecolor=SURF, gridspec_kw={"width_ratios": [1, 1.4]})
    style(ax)
    t0 = L[0][0]
    ax[0].plot([p[1] for p in L], [p[2] for p in L], color=BLUE, lw=1.8, label=f"{LN} (leader)")
    for n, F in paths_by.items():
        ax[0].plot([p[1] for p in F], [p[2] for p in F], color=col[n], lw=1.2, label=n)
    ax[0].set_aspect("equal"); ax[0].set_xlabel("East (m)", color=INK2); ax[0].set_ylabel("North (m)", color=INK2)
    ax[0].set_title("Paths (Gazebo truth)", loc="left", fontsize=11, fontweight="bold", color=INK)
    ax[0].legend(frameon=False, fontsize=9, labelcolor=INK2)
    for n, ser in series_by.items():
        ax[1].plot([x[0] - t0 for x in ser], [x[1] for x in ser], color=col[n], lw=1.3, label=n)
    if mv:
        ax[1].axvspan(mv[0] - t0, mv[1] - t0, color=GRID, alpha=0.6, lw=0, label="leader moving (scored)")
    ax[1].axhline(max_p95, color=INK2, lw=1, ls=(0, (5, 3)), label=f"acceptance (p95 <= {max_p95:g} m)")
    ax[1].set_xlabel("time (s)", color=INK2); ax[1].set_ylabel("horizontal formation error (m)", color=INK2)
    ax[1].set_title("Formation error over the radio link", loc="left", fontsize=11, fontweight="bold", color=INK)
    ax[1].legend(frameon=False, fontsize=9, labelcolor=INK2)
    fig.savefig(f"{log}/formation.png", dpi=140, facecolor=SURF, bbox_inches="tight")

    pts = [(n, r) for n, r in report["followers"].items() if "swarm_lat_p50_ms" in r and "mean_leader_distance_m" in r]
    if len(pts) > 1:
        fig, ax = plt.subplots(1, 2, figsize=(11, 4.2), facecolor=SURF)
        style(ax)
        for n, r in pts:
            d = r["mean_leader_distance_m"]
            ax[0].errorbar([d], [r["swarm_lat_p50_ms"]], yerr=[[0], [max(0, r["swarm_lat_p99_ms"] - r["swarm_lat_p50_ms"])]],
                           fmt="o", color=col[n], capsize=3, label=n)
            ax[1].plot([d], [r["swarm_loss_pct"]], "o", color=col[n], label=n)
        ax[0].set_xlabel("mean distance to leader (m)", color=INK2); ax[0].set_ylabel("swarm latency, p50 (bar to p99) (ms)", color=INK2)
        ax[0].set_title("Latency vs distance", loc="left", fontsize=11, fontweight="bold", color=INK)
        ax[1].set_xlabel("mean distance to leader (m)", color=INK2); ax[1].set_ylabel("swarm sequence loss (%)", color=INK2)
        ax[1].set_title("Loss vs distance", loc="left", fontsize=11, fontweight="bold", color=INK)
        ax[0].legend(frameon=False, fontsize=9, labelcolor=INK2)
        ax[1].set_ylim(0, max(1.0, 1.1 * max(r["swarm_loss_pct"] for _, r in pts)))
        fig.savefig(f"{log}/link_vs_distance.png", dpi=140, facecolor=SURF, bbox_inches="tight")
except Exception as e:
    print("plot skipped:", repr(e))
sys.exit(0 if ok_all else 1)
