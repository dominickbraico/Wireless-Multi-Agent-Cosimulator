#!/usr/bin/env python3
"""2D map of the inter-agent radio links: every agent pair, coloured by received signal strength.

  tools/plot_comm_map.py <log_dir> [--time T | --snapshots N] [--out PNG] [--rssi-range LO HI] [--curved auto|always|never]
  tools/plot_comm_map.py --config <experiment.yaml> [...]          static positions from the YAML (no run needed)

Agents are drawn top-down (x east, y north) at their ns-3 positions, with the path flown so far as a faint trace. Every pair
gets one line, dark green for a strong signal fading through yellow and orange to red as it weakens. A dashed line means the
receiver floor (preamble_detection_min_rssi_dbm) is not met, so no frame between that pair is detected at all.

The signal strength is the received power the configured propagation model gives at the plotted instant: tx power minus
the chained propagation losses, evaluated exactly as ns-3 does (LogDistance: no loss within the reference distance, then
ref_loss + 10*exponent*log10(d/ref)). It uses the 3D distance between antennas. Run logs supply the positions
(<log_dir>/ns3/pose_applied.csv, the poses ns-3 applied) and the experiment copy ns-3 saved next to them.

--time T       ns-3 time (s) of the snapshot (default: middle of the airborne period, or of the whole run)
--snapshots N  N panels evenly spread over the airborne period instead of one
--rssi-range   dBm mapped to red and to dark green (default: the receiver floor of the YAML, and -50 dBm)
--curved       draw pairs as arcs so agents in a straight line do not hide each other's links (auto: only if collinear)
"""
import argparse
import csv
import itertools
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrowPatch

# dark green (strong) -> green -> yellow -> orange -> red (weak)
CMAP = LinearSegmentedColormap.from_list("signal", ["#c40000", "#e8590c", "#f2c500", "#74b816", "#0b5d1e"])
DEFAULT_STRONG_DBM = -50.0


def rx_power_dbm(cfg, d):
    """Received power at distance d (m) after the configured chain of LogDistance losses, as ns-3 computes it."""
    p = float(cfg["network"]["wifi"]["tx_power_dbm"])
    for m in cfg["network"]["channel"]["loss"]:
        if m["type"] != "LogDistance":
            raise SystemExit(f"plot_comm_map: loss model {m['type']} is not supported by this tool")
        if d > m["ref_distance_m"]:
            p += -m["ref_loss_db"] - 10.0 * m["exponent"] * math.log10(d / m["ref_distance_m"])
        # at or inside the reference distance ns-3 returns the power unchanged
    return p


def floor_dbm(cfg):
    return float(cfg["network"]["wifi"].get("phy", {}).get("preamble_detection_min_rssi_dbm", -82.0))


def static_positions(cfg):
    pos = {}
    for a in cfg["agents"]:
        if a.get("initial_position") is not None:
            pos[a["name"]] = [float(v) for v in a["initial_position"]]
        else:
            off = a.get("antenna_offset") or [0, 0, 0]
            pos[a["name"]] = [float(a["spawn_pose"][i]) + float(off[i]) for i in range(3)]
    return pos


def load_tracks(log_dir, cfg):
    """-> {agent: (t[], xyz[][3])} from the poses ns-3 applied, or None when the run has no pose feed."""
    path = os.path.join(log_dir, "ns3", "pose_applied.csv")
    if not os.path.exists(path):
        return None
    names = [a["name"] for a in cfg["agents"]]
    rows = {n: ([], []) for n in names}
    with open(path) as f:
        for r in csv.DictReader(f):
            n = names[int(r["node"])]                       # ns-3 node i is the i-th agent of the YAML
            rows[n][0].append(float(r["t_ns3_s"]))
            rows[n][1].append([float(r["x"]), float(r["y"]), float(r["z"])])
    rows = {n: (np.array(t), np.array(p)) for n, (t, p) in rows.items() if t}
    return rows or None


def position_at(track, t):
    ts, ps = track
    return ps[max(0, int(np.searchsorted(ts, t, side="right")) - 1)]


def find_config(log_dir):
    for rel in ("ns3/experiment.yaml", "compose/experiment_run.yaml", "experiment_run.yaml"):
        p = os.path.join(log_dir, rel)
        if os.path.exists(p):
            return p
    raise SystemExit(f"plot_comm_map: no experiment YAML found in {log_dir} (pass --config)")


def airborne_window(tracks):
    ts = np.concatenate([t[(p[:, 2] > 1.0)] for t, p in tracks.values()]) if tracks else np.array([])
    if ts.size:
        return float(ts.min()), float(ts.max())
    every = np.concatenate([t for t, _ in tracks.values()])
    return float(every.min()), float(every.max())


def collinear(xy):
    c = xy - xy.mean(axis=0)
    s = np.linalg.svd(c, compute_uv=False)
    return len(s) < 2 or s[0] == 0 or s[1] / s[0] < 0.05


def draw_panel(ax, cfg, names, pos, tracks, t, norm, floor, curved, label_links=True):
    xy = np.array([pos[n][:2] for n in names])
    leader = names[0]
    try:
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "orchestrator"))
        from fleet import resolve
        leader = resolve(cfg)[0]["name"]
    except Exception:
        pass
    if tracks:
        for n in names:
            if n in tracks:
                ts, ps = tracks[n]
                m = ts <= t
                ax.plot(ps[m, 0], ps[m, 1], color="#9aa0a6", lw=0.8, alpha=0.6, zorder=1)
    use_arcs = curved == "always" or (curved == "auto" and collinear(xy))
    for (i, a), (j, b) in itertools.combinations(enumerate(names), 2):
        d = float(np.linalg.norm(np.array(pos[a]) - np.array(pos[b])))
        rssi = rx_power_dbm(cfg, d)
        dead = rssi < floor
        colour = CMAP(norm(rssi))
        rad = (0.16 * (j - i) * (1 if (i + j) % 2 == 0 else -1)) if use_arcs else 0.0
        ax.add_patch(FancyArrowPatch(xy[i], xy[j], arrowstyle="-", connectionstyle=f"arc3,rad={rad}",
                                     color=colour, lw=2.4, ls=(0, (4, 3)) if dead else "-", alpha=0.95, zorder=2))
        if label_links:
            mid = (xy[i] + xy[j]) / 2
            dx, dy = xy[j] - xy[i]
            mid = mid + 0.5 * rad * np.array([dy, -dx])           # midpoint of the quadratic arc matplotlib draws
            ax.text(mid[0], mid[1], f"{rssi:.0f}", fontsize=7, ha="center", va="center", zorder=4,
                    color="#202124", bbox=dict(boxstyle="round,pad=0.12", fc="white", ec=colour, lw=0.9, alpha=0.92))
    for n, (x, y) in zip(names, xy):
        ax.scatter([x], [y], s=150 if n == leader else 90, marker="*" if n == leader else "o", c="#1a73e8" if n == leader else "#3c4043",
                   edgecolors="white", linewidths=1.2, zorder=5)
        ax.annotate(n + (" (leader)" if n == leader else ""), (x, y), textcoords="offset points", xytext=(9, 7),
                    fontsize=8.5, fontweight="bold", color="#202124", zorder=6)
    pad = max(2.0, 0.12 * max(np.ptp(xy[:, 0]), np.ptp(xy[:, 1]), 1.0))
    ax.set_xlim(xy[:, 0].min() - pad - (0.15 * np.ptp(xy[:, 1]) if use_arcs else 0), xy[:, 0].max() + pad + (0.15 * np.ptp(xy[:, 1]) if use_arcs else 0))
    ax.set_ylim(xy[:, 1].min() - pad, xy[:, 1].max() + pad)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x east (m)")
    ax.set_ylabel("y north (m)")
    ax.grid(True, color="#e8eaed", lw=0.6)
    ax.set_axisbelow(True)
    if t is not None:
        ax.set_title(f"t = {t:.1f} s", fontsize=10)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("log_dir", nargs="?")
    ap.add_argument("--config")
    ap.add_argument("--time", type=float)
    ap.add_argument("--snapshots", type=int)
    ap.add_argument("--out")
    ap.add_argument("--rssi-range", nargs=2, type=float, metavar=("LO", "HI"))
    ap.add_argument("--curved", choices=["auto", "always", "never"], default="auto")
    a = ap.parse_args()
    if not a.log_dir and not a.config:
        ap.error("give a log directory or --config")
    cfg_path = a.config or find_config(a.log_dir)
    cfg = yaml.safe_load(open(cfg_path))
    names = [x["name"] for x in cfg["agents"]]
    tracks = load_tracks(a.log_dir, cfg) if a.log_dir else None
    floor = floor_dbm(cfg)
    lo, hi = a.rssi_range if a.rssi_range else (floor, DEFAULT_STRONG_DBM)
    norm = Normalize(vmin=lo, vmax=hi, clip=True)

    if tracks:
        t0, t1 = airborne_window(tracks)
        times = [a.time] if a.time is not None else (
            [t0 + (t1 - t0) * (k + 0.5) / a.snapshots for k in range(a.snapshots)] if a.snapshots else [(t0 + t1) / 2])
        # an agent that never got a pose keeps its YAML position
        fallback = static_positions(cfg)
        snaps = [(t, {n: (position_at(tracks[n], t) if n in tracks else fallback[n]) for n in names}) for t in times]
    else:
        snaps = [(None, static_positions(cfg))]

    n = len(snaps)
    fig, axes = plt.subplots(1, n, figsize=(6.4 * n + 1.4, 6.0), squeeze=False)
    for ax, (t, pos) in zip(axes[0], snaps):
        draw_panel(ax, cfg, names, pos, tracks, t, norm, floor, a.curved)
    sm = plt.cm.ScalarMappable(norm=norm, cmap=CMAP)
    cb = fig.colorbar(sm, ax=list(axes[0]), fraction=0.04 if n == 1 else 0.025, pad=0.02)
    cb.set_label("received signal strength (dBm)")
    cb.ax.axhline(floor, color="#202124", lw=1.0) if lo < floor < hi else None
    fig.legend(handles=[Line2D([0], [0], color=CMAP(0.95), lw=2.4, label="strong"),
                        Line2D([0], [0], color=CMAP(0.0), lw=2.4, label="weak"),
                        Line2D([0], [0], color=CMAP(0.0), lw=2.4, ls=(0, (4, 3)), label=f"below receiver floor ({floor:.0f} dBm): no link")],
               loc="lower center", ncol=3, frameon=False, fontsize=8.5)
    fig.suptitle(f"Inter-agent signal strength: {cfg['experiment']['name']}   ({len(names)} agents, {len(names) * (len(names) - 1) // 2} links; "
                 f"numbers are dBm)", fontsize=11)
    fig.subplots_adjust(bottom=0.13, top=0.88)
    out = a.out or os.path.join(a.log_dir or ".", "comm_map.png")
    fig.savefig(out, dpi=150)
    print(f"plot_comm_map: wrote {out}")


if __name__ == "__main__":
    main()
