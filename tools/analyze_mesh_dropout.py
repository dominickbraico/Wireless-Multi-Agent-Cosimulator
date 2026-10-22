#!/usr/bin/env python3
"""Mesh-dropout analysis: did the planned links drop and recover, as the model predicts and as the agents' own heartbeats show?

  analyze_mesh_dropout.py <log_dir> <experiment.yaml>
Inputs from a container run of the mesh-dropout scenario (docs/MESH_DROPOUT.md): mesh_plan.json (the plan flown), ns3/pair_rssi.csv (ns-3's
signal for every ordered pair, every link_log_interval_s), ns3/pose_applied.csv (ground-truth positions and the ns-3 <-> wall-clock time map),
mesh/mesh_events_<agent>.csv (UP / LOST per neighbour, from each agent's own heartbeat reception), flight logs and the ns-3 summary.

Two modes of the scenario (plan["mode"]): `excursion` (one wanderer, the rest hover) is checked against the planned dropped pairs; `roam` (every agent flies) is
checked against the modelled drops themselves, because which links drop depends on where both ends are at that moment. Pass criteria, fixed before the first run:
  flights    every agent reached its altitude, landed and disarmed; was within 2.0 m (horizontal) of its spawn point when its landing began; ended within 5.0 m of it
             after touchdown (changed 2026-10-21: the end position used to be held to 2.0 m, but on Baylands' sloping ground a vehicle can be bumped about 1 to 2 m after touchdown)
  modelled   the links to the planned dropped anchors fall below the receiver floor and later recover; every other link (to the kept
             anchors, and between anchors) never falls below it
  measured   (heartbeats, until the last agent has landed) in both directions of each dropped link: a LOST event followed by an UP event; LOST is detected within
             [-1.0 s, link_timeout + 2.0 s] of the modelled drop; UP follows the modelled recovery within [-1.0 s, 10.0 s];
             every LOST event falls where the model has the signal below the floor (no unexplained link loss)
  roam mode  modelled: at least one link falls below the floor for >= 3.0 s and recovers; every link is above the floor at the end; never more than
             max_dropped_links links below it at once. measured: for every modelled drop lasting at least link_timeout + 1 s, in both directions a LOST event within
             [-1.0 s, link_timeout + 2.0 s] of the drop and an UP event within [-1.0 s, 10.0 s] of the recovery; plus the no-unexplained-LOST rule below.
  validity   ns-3 lag within time.max_lag_ms; Gazebo RTF 5-s sliding mean >= time.min_rtf from take-off (see docs/ARCHITECTURE.md)
  isolation  no domain-0, XRCE or PX4-topic traffic on the emulated channel; the swarm domain (42) and the mesh topic are present
Writes mesh_dropout.png (signal and heartbeat timeline, trajectories), comm_map_dropout.png (map before / at the deepest dropout / after),
mesh_report.json. Exit 0 if every check passes.
"""
import csv
import json
import math
import os
import subprocess
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "orchestrator"))
sys.path.insert(0, HERE)
import mesh_plan
from timehealth import rtf_check

HOME_TOL_M, DETECT_MIN_S, DETECT_EXTRA_S, RECOVER_MIN_S, RECOVER_MAX_S = 2.0, -1.0, 2.0, -1.0, 10.0
END_TOL_M = 5.0                                                            # after touchdown: a sanity bound for a crash or fly-away, not a precision limit
MIN_OBSERVED_DROP_S = 3.0                                                  # roam mode: a modelled drop at least this long counts as a clear drop

log, cfg_f = sys.argv[1:3]
cfg = yaml.safe_load(open(cfg_f))
names = [a["name"] for a in cfg["agents"]]
ids = {a["name"]: a["px4_instance"] + 1 for a in cfg["agents"]}            # heartbeat sender_id = px4_instance + 1
by_id = {v: k for k, v in ids.items()}
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


plan_path = f"{log}/mesh_plan.json"
plan = json.load(open(plan_path)) if os.path.exists(plan_path) else mesh_plan.plan(cfg)
settings = plan["settings"]
timeout_s = float(settings["link_timeout_s"])
mode = plan.get("mode", "excursion")
wanderers = list(plan.get("wanderers", {}))
anchors = plan.get("anchors", [])
planned = {}                                                    # excursion mode: frozenset({wanderer, anchor}) -> True if it is planned to drop
if mode == "excursion":
    for w, wp in plan["wanderers"].items():
        for a in anchors:
            planned[frozenset((w, a))] = a in wp["excursion"]["dropped_anchors"]
    print(f"plan: excursion mode, seed {plan['seed']}, dropout range D = {plan['dropout_range_m']:.1f} m, "
          + "; ".join(f"{w} excursion drops {wp['excursion']['dropped_anchors']} at {wp['excursion']['distance_from_home_m']} m"
                      for w, wp in plan["wanderers"].items()))
else:
    print(f"plan: roam mode (every agent flies), seed {plan['seed']} ({plan['attempts']} candidate plans tried), dropout range D = {plan['dropout_range_m']:.1f} m, "
          f"flight {plan['t_total_s']:.0f} s; planned clear drops: "
          + "; ".join(f"{e['pair'][0]}-{e['pair'][1]} t={e['t_start_s']:.0f}-{e['t_end_s']:.0f} s (peak {e['peak_distance_m']:.0f} m)" for e in plan["nominal_events"]))

# ---- flights ----
for n in names:
    t = text(f"{log}/{n}_flight.log")
    check(f"{n} armed, took off and reached altitude", "Reached altitude" in t)
    check(f"{n} landed and disarmed", "Landed and disarmed" in t)

# ---- ground truth: positions with their wall-clock times; the ns-3 <-> wall time map ----
trk = {i: [] for i in range(len(names))}
for r in rows(f"{log}/ns3/pose_applied.csv"):
    try:
        trk[int(r["node"])].append((float(r["t_wall_apply_s"]), float(r["t_ns3_s"]), float(r["x"]), float(r["y"]), float(r["z"])))
    except (TypeError, ValueError, KeyError):
        pass
check("pose feed delivered every vehicle", all(len(v) > 100 for v in trk.values()), "/".join(str(len(v)) for v in trk.values()) + " samples")
T = {names[i]: np.array(v) for i, v in trk.items() if v}                # columns: wall, ns3, x, y, z
ref = T[names[0]]
ns3_to_wall = lambda t: np.interp(t, ref[:, 1], ref[:, 0])
def landing_started(n):
    """Wall-clock time at which this agent's flight node commanded the landing (from its log), or None."""
    for line in text(f"{log}/{n}_flight.log").splitlines():
        if "Trajectory complete, landing" in line:
            try:
                return float(line.split("]")[1].strip(" ["))
            except (IndexError, ValueError):
                return None
    return None

for n in names:
    sp = plan["spawn"][n]
    if n not in T:
        check(f"{n} was back within {HOME_TOL_M:g} m of its spawn point when landing began", False, "no poses")
        continue
    tl = landing_started(n)
    k = int(np.argmin(np.abs(T[n][:, 0] - tl))) if tl is not None else -1
    d_home = math.hypot(T[n][k, 2] - sp[0], T[n][k, 3] - sp[1])
    d_end = math.hypot(T[n][-1, 2] - sp[0], T[n][-1, 3] - sp[1])
    check(f"{n} was back within {HOME_TOL_M:g} m of its spawn point when landing began", tl is not None and d_home <= HOME_TOL_M, f"{d_home:.2f} m" if tl is not None else "no landing line in the flight log")
    check(f"{n} ended within {END_TOL_M:g} m of its spawn point after touchdown", d_end <= END_TOL_M, f"{d_end:.2f} m")
    if d_end > HOME_TOL_M and d_home <= HOME_TOL_M:
        print(f"note  {n}: hovered {d_home:.2f} m from its spawn point when landing began but ended {d_end:.2f} m away: moved after touchdown (contact with the uneven ground in the simulator)")

# ---- modelled links: ns-3's own signal for every pair ----
pair = {}                                                      # (tx, rx) -> list of (t_ns3, rssi, below)
for r in rows(f"{log}/ns3/pair_rssi.csv"):
    pair.setdefault((r["tx"], r["rx"]), []).append((float(r["t_s"]), float(r["rssi_dbm"]), int(r["below_floor"])))
check("ns-3 pair log present", bool(pair), "ns3/pair_rssi.csv")
modelled = {}                                                  # frozenset -> dict(first_drop_t, recover_t, intervals)
report = {"seed": plan["seed"], "dropout_range_m": plan["dropout_range_m"], "links": {}, "thresholds": {
    "home_tol_m": HOME_TOL_M, "end_tol_m": END_TOL_M, "detect_s": [DETECT_MIN_S, timeout_s + DETECT_EXTRA_S], "recover_s": [RECOVER_MIN_S, RECOVER_MAX_S]}}
for (tx, rx), v in pair.items():
    key = frozenset((tx, rx))
    if tx > rx:
        continue                                               # one direction: the model is symmetric (equal power and gains)
    t = np.array([x[0] for x in v]); below = np.array([x[2] for x in v]); rssi = np.array([x[1] for x in v])
    idx = np.flatnonzero(below)
    ivs, start = [], None                                      # (t_start, t_recovered or None) for every run of samples below the floor
    for i in range(len(t)):
        if below[i] and start is None:
            start = float(t[i])
        elif not below[i] and start is not None:
            ivs.append((start, float(t[i]))); start = None
    if start is not None:
        ivs.append((start, None))
    modelled[key] = {"t": t, "below": below, "rssi": rssi, "ever": bool(idx.size), "intervals": ivs,
                     "first_drop": float(t[idx[0]]) if idx.size else None,
                     "recover": float(t[idx[-1] + 1]) if idx.size and idx[-1] + 1 < len(t) else None,
                     "recovered": bool(idx.size and below[-1] == 0)}
for key, m in sorted(modelled.items(), key=lambda kv: sorted(kv[0])):
    a, b = sorted(key)
    want_drop = planned.get(key, False)
    label = f"{a} - {b}"
    if mode == "roam":
        report["links"][label] = {"modelled_drops_ns3_s": [list(iv) for iv in m["intervals"]], "min_rssi_dbm": float(m["rssi"].min())}
        continue
    if key in planned:
        if want_drop:
            check(f"modelled signal {label}: falls below the floor and recovers (planned)", m["ever"] and m["recovered"],
                  f"below the floor from t={m['first_drop']:.1f} s, back at t={m['recover']:.1f} s (ns-3 time)" if m["ever"] else "never below the floor")
        else:
            check(f"modelled signal {label}: stays above the floor (planned)", not m["ever"], f"min {m['rssi'].min():.1f} dBm")
    else:
        check(f"modelled signal {label}: stays above the floor (anchors)", not m["ever"], f"min {m['rssi'].min():.1f} dBm")
    report["links"][label] = {"planned_drop": bool(want_drop), "modelled_first_drop_ns3_s": m["first_drop"], "modelled_recover_ns3_s": m["recover"],
                              "min_rssi_dbm": float(m["rssi"].min())}

if mode == "roam" and modelled:
    clear = [(k, iv) for k, m in modelled.items() for iv in m["intervals"] if iv[1] is not None and iv[1] - iv[0] >= MIN_OBSERVED_DROP_S]
    check(f"at least one link falls below the floor for >= {MIN_OBSERVED_DROP_S:g} s and recovers (modelled)", len(clear) >= 1,
          f"{len(clear)} clear drops: " + "; ".join(f"{'-'.join(sorted(k))} {iv[0]:.0f}-{iv[1]:.0f} s" for k, iv in clear) + f" (planned {len(plan['nominal_events'])})")
    check("every link is above the floor again at the end of the flight (modelled)", all(m["below"][-1] == 0 for m in modelled.values()))
    nmin = min(len(m["below"]) for m in modelled.values())
    simult = np.sum([m["below"][:nmin] for m in modelled.values()], axis=0)
    cap = int(plan["settings"]["max_dropped_links"])
    check(f"never more than {cap} links below the floor at once (modelled)", int(simult.max()) <= cap, f"at most {int(simult.max())} at once")
    report["max_links_down_at_once"] = int(simult.max())

# ---- measured links: the agents' own heartbeat reception ----
events = []                                                    # (t_wall, agent, peer, event, silence)
for n in names:
    for r in rows(f"{log}/mesh/mesh_events_{n}.csv"):
        events.append((float(r["t_s"]), n, by_id.get(int(r["peer_id"]), r["peer_id"]), r["event"], float(r["silence_s"])))
events.sort()
done_times = [os.path.getmtime(f"{log}/{n}.done") for n in names if os.path.exists(f"{log}/{n}.done")]
t_end = max(done_times) + 1.5 if len(done_times) == len(names) else float("inf")      # the harness stops ns-3 after this: every link then "drops"
events = [e for e in events if e[0] <= t_end]
check("heartbeat logs present for every agent", all(os.path.exists(f"{log}/mesh/mesh_events_{n}.csv") for n in names))
print("link events (seconds after the first event):")
t_first = events[0][0] if events else 0.0
for t, a, p, e, sil in events:
    if e == "LOST" or sil > 0:
        print(f"  t={t - t_first:7.1f}  {a} {e:4s} {p}" + (f"  (silent {sil:.1f} s)" if sil else ""))
for key, want_drop in planned.items():
    a, b = sorted(key)
    m = modelled.get(key)
    if m is None:
        continue
    for rx, tx in ((a, b), (b, a)):
        ev = [e for e in events if e[1] == rx and e[2] == tx]
        lost = [e for e in ev if e[3] == "LOST"]
        if not want_drop:
            continue                                       # covered by the unexplained-LOST check below
        if not (m["ever"] and lost):
            check(f"measured {rx} <- {tx}: LOST then UP", False, "no LOST event" if m["ever"] else "no modelled drop")
            continue
        drop_wall = float(ns3_to_wall(m["first_drop"]))
        l0 = lost[0][0]
        detect = l0 - drop_wall
        ups_after = [e for e in ev if e[3] == "UP" and e[0] > l0]
        rec_model = float(ns3_to_wall(m["recover"])) if m["recover"] is not None else None
        recover = (ups_after[0][0] - rec_model) if ups_after and rec_model is not None else None
        good_detect = DETECT_MIN_S <= detect <= timeout_s + DETECT_EXTRA_S
        good_rec = recover is not None and RECOVER_MIN_S <= recover <= RECOVER_MAX_S
        check(f"measured {rx} <- {tx}: LOST then UP, detected {detect:+.1f} s after the modelled drop, back {('%+.1f' % recover) if recover is not None else 'never'} s after the modelled recovery",
              good_detect and good_rec)
        report["links"][f"{a} - {b}"].setdefault("measured", {})[f"{rx}<-{tx}"] = {"detect_delay_s": round(detect, 2), "recover_delay_s": None if recover is None else round(recover, 2),
                                                                                  "lost_events": len(lost)}
if mode == "roam":
    for key, m in sorted(modelled.items(), key=lambda kv: sorted(kv[0])):
        a, b = sorted(key)
        for k, (d0, d1) in enumerate(m["intervals"]):
            if d1 is None or d1 - d0 < timeout_s + 1.0:
                continue                                       # too short for the heartbeat timeout to be required to notice
            w0, w1 = float(ns3_to_wall(d0)), float(ns3_to_wall(d1))
            for rx, tx in ((a, b), (b, a)):
                ev = [e for e in events if e[1] == rx and e[2] == tx]
                lost = [e for e in ev if e[3] == "LOST" and w0 + DETECT_MIN_S <= e[0] <= w0 + timeout_s + DETECT_EXTRA_S]
                if not lost:
                    check(f"measured {rx} <- {tx}: LOST within the window after the modelled drop at ns-3 t={d0:.0f} s", False, "no LOST event")
                    continue
                ups = [e for e in ev if e[3] == "UP" and e[0] > lost[0][0]]
                recover = (ups[0][0] - w1) if ups else None
                check(f"measured {rx} <- {tx}: LOST then UP, detected {lost[0][0] - w0:+.1f} s after the modelled drop (ns-3 t={d0:.0f}-{d1:.0f} s), back "
                      f"{('%+.1f' % recover) if recover is not None else 'never'} s after the modelled recovery",
                      recover is not None and RECOVER_MIN_S <= recover <= RECOVER_MAX_S)
                report["links"][f"{a} - {b}"].setdefault("measured", {})[f"{rx}<-{tx}@{d0:.0f}s"] = {
                    "detect_delay_s": round(lost[0][0] - w0, 2), "recover_delay_s": None if recover is None else round(recover, 2)}
# every LOST event must be explained by the modelled signal: below the floor at some moment in the silence (plus 1 s for the sample interval)
unexplained = []
for t, a, p, e, sil in events:
    if e != "LOST":
        continue
    m = modelled.get(frozenset((a, p)))
    if m is None:
        continue
    lo, hi = float(np.interp(t - sil - 1.0, ref[:, 0], ref[:, 1])), float(np.interp(t, ref[:, 0], ref[:, 1]))
    sel = (m["t"] >= lo - 0.5) & (m["t"] <= hi)
    if not (m["below"][sel].any()):
        unexplained.append(f"{a} lost {p} at ns-3 t={hi:.1f} s")
n_lost = sum(1 for e in events if e[3] == "LOST")
check("every LOST event coincides with a modelled drop below the floor (no unexplained link loss)", not unexplained,
      f"{len(unexplained)} of {n_lost} LOST events unexplained, e.g. {unexplained[:2]}" if unexplained else f"{n_lost} LOST events, all explained")

# full isolation (plans with min_isolation_s > 0): from its own heartbeat log, one agent had EVERY neighbour LOST at the same time
if plan.get("nominal_isolations"):
    best = (0.0, None)
    for n in names:
        peers = [p for p in names if p != n]
        state = {p: False for p in peers}                                           # True = LOST
        t_all = None
        for t, a, p, ev, _ in events:
            if a != n:
                continue
            state[p] = ev == "LOST"
            if all(state.values()) and t_all is None:
                t_all = t
            elif not all(state.values()) and t_all is not None:
                if t - t_all > best[0]:
                    best = (t - t_all, n)
                t_all = None
    iso_plan = ", ".join(f"{i['agent']} at plan t={i['t_start_s']:.0f}-{i['t_end_s']:.0f} s" for i in plan["nominal_isolations"])
    check("one agent had every neighbour LOST at the same time (fully disconnected), measured from its own heartbeats", best[1] is not None,
          f"{best[1]} for {best[0]:.1f} s (planned: {iso_plan})" if best[1] else f"never (planned: {iso_plan})")

# heartbeat statistics while the links were up
lat, gaps = [], 0
for n in names:
    for r in rows(f"{log}/mesh/mesh_rx_{n}.csv"):
        lat.append(float(r["latency_ms"])); gaps += int(r["gap"])
if lat:
    pct = lambda v, q: sorted(v)[min(len(v) - 1, int(round(q * (len(v) - 1))))]
    print(f"heartbeats: {len(lat)} received, latency p50 {pct(lat, .5):.2f} ms, p99 {pct(lat, .99):.2f} ms, {gaps} lost in sequence gaps (includes the dropouts)")
    report["heartbeats"] = {"received": len(lat), "latency_p50_ms": round(pct(lat, .5), 2), "latency_p99_ms": round(pct(lat, .99), 2), "sequence_gaps": gaps}

# ---- validity ----
try:
    summ = json.load(open(f"{log}/ns3/summary.json"))
    check("ns-3 real-time lag within limit", summ["valid_ns3_lag"], f"max {summ['ns3_max_lag_ms']} ms")
except FileNotFoundError:
    check("ns-3 summary.json written (clean shutdown)", False)
rt = [(float(r["t_wall_s"]), float(r["rtf"])) for r in rows(f"{log}/bridge/rtf.csv")]
if rt and ref.size:
    win, rmin = cfg["time"].get("rtf_window_s", 5.0), cfg["time"]["min_rtf"]
    t_pose0 = ref[0, 0] + 5
    go = f"{log}/go"
    t_flight = max(t_pose0, os.path.getmtime(go)) if os.path.exists(go) else t_pose0
    rc = rtf_check(rt, t_flight, ref[-1, 0], rmin, win)
    check(f"Gazebo RTF {win:g}-s sliding mean >= {rmin}", rc["ok"],
          f"worst window mean {rc['worst_window_mean']:.3f}, worst sample {rc['worst_sample']:.2f}" if rc["n"] else "no samples")
    report["rtf_worst_window_mean"] = rc["worst_window_mean"]

# ---- radio isolation ----
def count(pcap, flt, grep=None):
    cmd = ["tcpdump", "-Z", "root", "-r", pcap, "-nn"] + (["-A"] if grep else []) + [flt]
    out = subprocess.run(cmd, capture_output=True, text=True).stdout
    return sum(1 for l in out.splitlines() if grep in l) if grep else len([l for l in out.splitlines() if l.strip()])

pcap = f"{log}/ns3/wifi-0-0.pcap"
if os.path.exists(pcap):
    dom0, xrce, fmu = count(pcap, "udp and portrange 7400-7650"), count(pcap, "udp and port 8888"), count(pcap, "udp", grep="fmu/")
    swarm, mesh = count(pcap, "udp and portrange 17900-18150"), count(pcap, "udp", grep="mesh")
    check("no local-domain (0) RTPS ever on the emulated channel", dom0 == 0, f"{dom0} frames")
    check("no XRCE-DDS (port 8888) traffic on the emulated channel", xrce == 0, f"{xrce} frames")
    check("no PX4 topic names (fmu/...) anywhere on the emulated channel", fmu == 0, f"{fmu} frames")
    check("swarm domain (42) RTPS present, mentioning the mesh topic", swarm > 0 and mesh > 0, f"{swarm} frames, {mesh} mention 'mesh'")
else:
    check("ns-3 pcap present", False)

json.dump(report, open(f"{log}/mesh_report.json", "w"), indent=1)

# ---- plots ----
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    floor = float(cfg["network"]["wifi"].get("phy", {}).get("preamble_detection_min_rssi_dbm", -82.0))
    t0 = ref[0, 1]
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 11), gridspec_kw={"height_ratios": [3, 2, 4]})
    if mode == "roam":
        plot_pairs = [(x, y) for i, x in enumerate(names) for y in names[i + 1:]]
    else:
        plot_pairs = [(w, a) for w in wanderers for a in anchors]
    palette = ["#2a78d6", "#2f9e6b", "#c2417a", "#d4a017", "#8a5cc2", "#e8590c", "#4a90a4", "#7a7a2c"]
    pcol = {frozenset(pr): palette[i % len(palette)] for i, pr in enumerate(plot_pairs)}
    pair_rows = rows(f"{log}/ns3/pair_rssi.csv")
    for x, y in plot_pairs:
        m = modelled.get(frozenset((x, y)))
        if m is not None:
            ax1.plot(m["t"] - t0, m["rssi"], color=pcol[frozenset((x, y))], lw=1.8, label=f"{x} - {y}")
    ax1.axhline(floor, color="#b00020", lw=1.2, ls="--"); ax1.text(0.01, floor + 0.6, f"receiver floor {floor:.0f} dBm: below it a link carries no frames", transform=ax1.get_yaxis_transform(), color="#b00020", fontsize=8)
    ax1.set_ylabel("modelled signal (dBm)")
    ax1.set_title(("Every link" if mode == "roam" else "Wanderer - anchor links") + ": modelled signal (lines) and what the agents' heartbeats saw (markers)")
    for t, a, p, e, sil in events:
        col = pcol.get(frozenset((a, p)))
        if col is None:
            continue
        tn = float(np.interp(t, ref[:, 0], ref[:, 1])) - t0
        ax1.plot([tn], [floor - 2.5 if e == "LOST" else floor - 5.0], marker="v" if e == "LOST" else "^", color=col, ms=7, ls="none")
    ax1.plot([], [], marker="v", color="grey", ls="none", label="LOST"); ax1.plot([], [], marker="^", color="grey", ls="none", label="UP")
    ax1.legend(loc="upper center", bbox_to_anchor=(0.5, -0.06), fontsize=8, ncol=4, frameon=False)
    for x, y in plot_pairs:
        rr = [r for r in pair_rows if r["tx"] == x and r["rx"] == y]
        ax2.plot([float(r["t_s"]) - t0 for r in rr], [float(r["distance_m"]) for r in rr], color=pcol[frozenset((x, y))], lw=1.5)
    ax2.axhline(plan["dropout_range_m"], color="#b00020", ls="--", lw=1.2)
    ax2.text(0.01, plan["dropout_range_m"] + 1, f"dropout range D = {plan['dropout_range_m']:.1f} m (3-D)", transform=ax2.get_yaxis_transform(), color="#b00020", fontsize=8)
    ax2.set_ylabel("distance (m)"); ax2.set_xlabel("ns-3 time (s)")
    for a in names:
        c = T[a]
        ax3.plot(c[:, 2], c[:, 3], lw=2.2 if a in wanderers else 1.4, label=a + (" (wanders)" if a in wanderers else " (anchor)" if mode == "excursion" else ""))
        ax3.scatter([plan["spawn"][a][0]], [plan["spawn"][a][1]], s=50, marker="o", edgecolors="k", zorder=5)
    if mode == "excursion":
        for w, wp in plan["wanderers"].items():
            ex = wp["excursion"]["xy"]
            ax3.scatter([ex[0]], [ex[1]], s=140, marker="X", color="#b00020", zorder=6, label=f"excursion point ({w})")
            ax3.add_patch(plt.Circle(tuple(plan["spawn"][anchors[0]][:2]), plan["dropout_range_m"], fill=False, ls=":", color="#b00020", alpha=0.5))
        ax3.set_title(f"Paths (dots = spawn points; dotted circle = dropout range around {anchors[0]})")
    else:
        ax3.set_title("Paths (dots = spawn points; every agent flies a seeded random path and returns)")
    ax3.set_aspect("equal", adjustable="datalim"); ax3.grid(True, color="#e8eaed"); ax3.legend(fontsize=8); ax3.set_xlabel("x east (m)"); ax3.set_ylabel("y north (m)")
    fig.tight_layout(); fig.savefig(f"{log}/mesh_dropout.png", dpi=130); print(f"wrote {log}/mesh_dropout.png")
except Exception as e:                                        # the checks above are the result; a plotting problem must not hide them
    print(f"note  mesh_dropout.png not written: {e}")

# comm map at three moments: before the first drop, at the deepest dropout, after the last recovery
try:
    drops = [m for m in modelled.values() if m["ever"]]
    if drops:
        t_a, t_b = min(m["first_drop"] for m in drops), max(m["recover"] or m["t"][-1] for m in drops)
        allt = next(iter(modelled.values()))["t"]
        count_below = np.sum([m["below"] for m in modelled.values()], axis=0)
        t_deep = float(allt[int(np.argmax(count_below))])
        air0 = float(allt[0])
        times = [max(air0, t_a - 8.0), t_deep, min(float(allt[-1]), t_b + 8.0)]
        subprocess.run([sys.executable, os.path.join(HERE, "plot_comm_map.py"), log, "--config", cfg_f, "--times", *[f"{t:.1f}" for t in times],
                        "--out", f"{log}/comm_map_dropout.png"], check=False)
except Exception as e:
    print(f"note  comm_map_dropout.png not written: {e}")
sys.exit(0 if ok_all else 1)
