#!/usr/bin/env python3
"""2D map of the inter-agent radio links: every agent pair, coloured by received signal strength.

  tools/plot_comm_map.py <log_dir> [--time T | --snapshots N] [--out PNG] [--rssi-range LO HI] [--curved auto|always|never]
  tools/plot_comm_map.py --config <experiment.yaml> [...]          static positions from the YAML (no run needed)
  tools/plot_comm_map.py <log_dir> --live                          live window that follows a running experiment

Agents are drawn top-down (x east, y north) at their ns-3 positions, with the path flown so far as a faint trace. Every pair
gets one line, dark green for a strong signal fading through yellow and orange to red as it weakens. A dashed line means the
receiver floor (preamble_detection_min_rssi_dbm) is not met, so no frame between that pair is detected at all.

The signal strength is read from ns-3: the scenario evaluates its own channel loss chain for every ordered pair of agents on the
current node positions and logs it (<log_dir>/ns3/pair_rssi.csv: t_s, tx, rx, distance_m, rssi_dbm, below_floor, every
network.link_log_interval_s). It is the power a frame from tx would arrive with at rx at that moment, whether or not the pair ever
talks, and it follows any change of topology or loss model. If the two directions of a pair differ, the weaker one is drawn. Runs
without that log (older runs, or --config alone) fall back to the same LogDistance formula computed here from the YAML; the figure
says which source it used. Positions come from <log_dir>/ns3/pose_applied.csv and the experiment copy ns-3 saved next to it.

--time T | --snapshots N] [--out PNG] [--rssi-range LO HI] [--curved auto|always|never]
  tools/plot_comm_map.py --config <experiment.yaml> [...]          static positions from the YAML (no run needed)
  tools/plot_comm_map.py <log_dir> --live                          live window that follows a running experiment

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
--live         open a window and redraw about twice a second from the poses ns-3 is applying right now; follows a run that
               has not started yet (waits for the log directory). Close the window to stop. Needs a display and Tk.
--curved       draw pairs as arcs so agents in a straight line do not hide each other's links (auto: only if collinear)
"""
import argparse
import csv
import itertools
import math
import os
import sys

import matplotlib
matplotlib.use("TkAgg" if "--live" in sys.argv else "Agg")   # --live needs a window
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


class PairLog:
    """Per-pair signal samples from ns-3 (pair_rssi.csv). Incremental, so the live view can keep feeding it."""

    def __init__(self, path):
        self.path, self.f, self.partial = path, None, ""
        self.times, self.ticks = [], []                 # sample times; {(tx, rx): rssi_dbm} per sample time

    def poll(self):
        if self.f is None:
            if not os.path.exists(self.path):
                return
            self.f = open(self.path)
            self.f.readline()                           # header
        chunk = self.partial + self.f.read()
        lines = chunk.split("\n")
        self.partial = lines.pop()
        for line in lines:
            c = line.split(",")
            if len(c) < 6:
                continue
            t = float(c[0])
            if not self.times or t != self.times[-1]:
                self.times.append(t)
                self.ticks.append({})
            self.ticks[-1][(c[1], c[2])] = float(c[4])

    def available(self):
        return bool(self.ticks)

    def at(self, t=None):
        """The sample at or before time t (the latest one when t is None); a sample still being written is skipped."""
        if not self.ticks:
            return {}
        k = len(self.ticks) - 1 if t is None else max(0, int(np.searchsorted(self.times, t, side="right")) - 1)
        if k == len(self.ticks) - 1 and k > 0 and len(self.ticks[k]) < len(self.ticks[k - 1]):
            k -= 1
        return self.ticks[k]


def make_rssi_of(cfg, sample):
    """-> rssi_of(a, b, distance): ns-3's value for the pair (weaker direction) when the sample has it, else the YAML formula."""
    def rssi_of(a, b, d):
        v = [sample[k] for k in ((a, b), (b, a)) if k in sample]
        return (min(v), True) if v else (rx_power_dbm(cfg, d), False)
    return rssi_of


def draw_panel(ax, cfg, names, pos, tracks, t, norm, floor, curved, rssi_of, label_links=True):
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
    used_ns3 = True
    for (i, a), (j, b) in itertools.combinations(enumerate(names), 2):
        d = float(np.linalg.norm(np.array(pos[a]) - np.array(pos[b])))
        rssi, from_ns3 = rssi_of(a, b, d)
        used_ns3 = used_ns3 and from_ns3
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
    pts = [xy]
    if tracks:                                           # keep the whole trace in view, not just the current positions
        pts += [ps[ts <= t][:, :2] for ts, ps in (tracks[n] for n in names if n in tracks) if (ts <= t).any()]
    allp = np.vstack(pts)
    span = max(np.ptp(allp[:, 0]), np.ptp(allp[:, 1]), 1.0)
    pad = max(2.0, 0.14 * span)
    arc = 0.15 * np.ptp(xy[:, 1]) if use_arcs else 0.0   # collinear agents: the arcs bulge sideways
    ax.set_xlim(allp[:, 0].min() - pad - arc, allp[:, 0].max() + pad + arc)
    ax.set_ylim(allp[:, 1].min() - pad, allp[:, 1].max() + pad)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x east (m)")
    ax.set_ylabel("y north (m)")
    ax.grid(True, color="#e8eaed", lw=0.6)
    ax.set_axisbelow(True)
    if t is not None:
        ax.set_title(f"t = {t:.1f} s", fontsize=10)
    return used_ns3


class PoseTail:
    """Incrementally reads <log_dir>/ns3/pose_applied.csv while ns-3 appends to it."""

    def __init__(self, path, names):
        self.path, self.names, self.f, self.partial = path, names, None, ""
        self.rows = {n: ([], []) for n in names}
        self.latest = {}
        self.t = 0.0

    def poll(self):
        if self.f is None:
            if not os.path.exists(self.path):
                return
            self.f = open(self.path)
            self.f.readline()                                  # header
        chunk = self.partial + self.f.read()
        lines = chunk.split("\n")
        self.partial = lines.pop()                              # an unfinished last line waits for the next poll
        for line in lines:
            c = line.split(",")
            if len(c) < 7:
                continue
            n = self.names[int(c[1])]
            self.rows[n][0].append(float(c[0]))
            self.rows[n][1].append((float(c[4]), float(c[5]), float(c[6])))
            self.latest[n] = [float(c[4]), float(c[5]), float(c[6])]
            self.t = max(self.t, float(c[0]))

    def tracks(self, keep=4000):
        return {n: (np.array(t[-keep:]), np.array(p[-keep:])) for n, (t, p) in self.rows.items() if t}


def source_label(from_ns3):
    return ("signal read from ns-3's propagation model (pair_rssi.csv)" if from_ns3
            else "signal estimated from the YAML's LogDistance model (no ns-3 pair log)")


def live(a, cfg, names, norm, floor):
    for backend in ("TkAgg", "QtAgg"):
        try:
            plt.switch_backend(backend)
            break
        except Exception:
            continue
    from matplotlib.animation import FuncAnimation
    tail = PoseTail(os.path.join(a.log_dir, "ns3", "pose_applied.csv"), names)
    pairs = PairLog(os.path.join(a.log_dir, "ns3", "pair_rssi.csv"))
    fallback = static_positions(cfg)
    fig = plt.figure(figsize=(8.2, 7.2))
    ax = fig.add_axes([0.08, 0.14, 0.72, 0.72])
    cax = fig.add_axes([0.84, 0.14, 0.02, 0.72])
    cb = fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap=CMAP), cax=cax)
    cb.set_label("received signal strength (dBm)")
    fig.legend(handles=[Line2D([0], [0], color=CMAP(0.95), lw=2.4, label="strong"), Line2D([0], [0], color=CMAP(0.0), lw=2.4, label="weak"),
                        Line2D([0], [0], color=CMAP(0.0), lw=2.4, ls=(0, (4, 3)), label=f"below receiver floor ({floor:.0f} dBm)")],
               loc="lower center", ncol=3, frameon=False, fontsize=8.5)
    title = fig.suptitle(f"Live inter-agent signal strength: {cfg['experiment']['name']}", fontsize=11)
    try:
        fig.canvas.manager.set_window_title("Inter-agent communications")
    except Exception:
        pass

    def frame(_):
        tail.poll()
        pairs.poll()
        pos = {n: tail.latest.get(n, fallback[n]) for n in names}
        ax.clear()
        from_ns3 = draw_panel(ax, cfg, names, pos, tail.tracks() or None, tail.t if tail.latest else None, norm, floor, a.curved,
                              make_rssi_of(cfg, pairs.at()))
        title.set_text(f"Live inter-agent signal strength: {cfg['experiment']['name']}\n" + source_label(from_ns3 and pairs.available()))
        if not tail.latest:
            ax.set_title("waiting for poses from ns-3 ...", fontsize=10)
        return []

    anim = FuncAnimation(fig, frame, interval=500, cache_frame_data=False)
    plt.show()
    return anim


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("log_dir", nargs="?")
    ap.add_argument("--config")
    ap.add_argument("--time", type=float)
    ap.add_argument("--snapshots", type=int)
    ap.add_argument("--out")
    ap.add_argument("--rssi-range", nargs=2, type=float, metavar=("LO", "HI"))
    ap.add_argument("--curved", choices=["auto", "always", "never"], default="auto")
    ap.add_argument("--live", action="store_true")
    a = ap.parse_args()
    if not a.log_dir and not a.config:
        ap.error("give a log directory or --config")
    if a.live and not a.log_dir:
        ap.error("--live follows a run: give its log directory")
    if a.live and not a.config:                                # the log directory may not exist yet: wait for the YAML
        import time
        for _ in range(240):
            try:
                a.config = find_config(a.log_dir)
                break
            except SystemExit:
                time.sleep(0.5)
    cfg_path = a.config or find_config(a.log_dir)
    cfg = yaml.safe_load(open(cfg_path))
    names = [x["name"] for x in cfg["agents"]]
    tracks = load_tracks(a.log_dir, cfg) if a.log_dir else None
    floor = floor_dbm(cfg)
    lo, hi = a.rssi_range if a.rssi_range else (floor, DEFAULT_STRONG_DBM)
    norm = Normalize(vmin=lo, vmax=hi, clip=True)
    if a.live:
        live(a, cfg, names, norm, floor)
        return

    if tracks:
        t0, t1 = airborne_window(tracks)
        times = [a.time] if a.time is not None else (
            [t0 + (t1 - t0) * (k + 0.5) / a.snapshots for k in range(a.snapshots)] if a.snapshots else [(t0 + t1) / 2])
        # an agent that never got a pose keeps its YAML position
        fallback = static_positions(cfg)
        snaps = [(t, {n: (position_at(tracks[n], t) if n in tracks else fallback[n]) for n in names}) for t in times]
    else:
        snaps = [(None, static_positions(cfg))]

    pairs = PairLog(os.path.join(a.log_dir, "ns3", "pair_rssi.csv")) if a.log_dir else None
    if pairs:
        pairs.poll()
    n = len(snaps)
    fig, axes = plt.subplots(1, n, figsize=(6.4 * n + 1.4, 6.6), squeeze=False)
    from_ns3 = True
    for ax, (t, pos) in zip(axes[0], snaps):
        sample = pairs.at(t) if pairs and pairs.available() else {}
        from_ns3 &= draw_panel(ax, cfg, names, pos, tracks, t, norm, floor, a.curved, make_rssi_of(cfg, sample))
    sm = plt.cm.ScalarMappable(norm=norm, cmap=CMAP)
    fig.subplots_adjust(left=0.10 / n ** 0.5, right=0.87, bottom=0.14, top=0.88, wspace=0.28)
    cax = fig.add_axes([0.895, 0.14, 0.014, 0.74])
    cb = fig.colorbar(sm, cax=cax)
    cb.set_label("received signal strength (dBm)")
    cb.ax.axhline(floor, color="#202124", lw=1.0) if lo < floor < hi else None
    fig.legend(handles=[Line2D([0], [0], color=CMAP(0.95), lw=2.4, label="strong"),
                        Line2D([0], [0], color=CMAP(0.0), lw=2.4, label="weak"),
                        Line2D([0], [0], color=CMAP(0.0), lw=2.4, ls=(0, (4, 3)), label=f"below receiver floor ({floor:.0f} dBm): no link")],
               loc="lower center", ncol=3, frameon=False, fontsize=8.5)
    fig.suptitle(f"Inter-agent signal strength: {cfg['experiment']['name']}   ({len(names)} agents, {len(names) * (len(names) - 1) // 2} links; "
                 f"numbers are dBm)\n" + source_label(from_ns3), fontsize=11)
    out = a.out or os.path.join(a.log_dir or ".", "comm_map.png")
    fig.savefig(out, dpi=150)
    print(f"plot_comm_map: wrote {out}")


if __name__ == "__main__":
    main()
