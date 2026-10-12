#!/usr/bin/env python3
"""Distance-sweep analysis: per-step link statistics, comparison with the pre-computed
prediction, acceptance checks, and plots.

  analyze_sweep.py <log_dir> <experiment.yaml> <prediction.json>
Reads steps.json, ping.txt, udp_tx.csv, udp_rx.csv, swarm_follower.csv, ns3/rssi.csv,
ns3/summary.json, bridge/rtf.csv. Writes sweep_stats.csv, sweep.png, sweep_report.json.
Exit 0 if every check passes.
"""
import csv
import json
import math
import os
import re
import statistics as st
import sys

import yaml

log, cfg_f, pred_f = sys.argv[1:4]
cfg = yaml.safe_load(open(cfg_f))
pred = json.load(open(pred_f))
sw = json.load(open(f"{log}/steps.json"))
SETTLE = 2.0   # s ignored after each move (queue drain, ARP, route settle)
GUARD = 0.6    # s ignored before the next move (MAC queue can hold a packet up to 500 ms past it)
steps = sw["steps"]
ok_all = True


def check(name, cond, detail=""):
    global ok_all
    print(("PASS  " if cond else "FAIL  ") + name + (f"  ({detail})" if detail else ""))
    ok_all &= bool(cond)


def pct(vals, q):
    v = sorted(vals)
    return v[min(len(v) - 1, int(round(q * (len(v) - 1))))] if v else float("nan")


# ---- windows ----
wins, prev = [], sw["t_window_start"]
x_max_idx = max(range(len(steps)), key=lambda i: steps[i]["pos"][0])
for i, s in enumerate(steps):
    wins.append({"i": i, "d": s["pos"][0], "t0": prev + SETTLE, "t1": s["t_wall_end"] - GUARD, "t_move": prev,
                 "pass": "fwd" if i <= x_max_idx else "ret"})
    prev = s["t_wall_end"]

# ---- load data ----
def rows(path):
    return list(csv.DictReader(open(path))) if os.path.exists(path) else []

rssi = [(float(r["t_wall_s"]), float(r["signal_dbm"])) for r in rows(f"{log}/ns3/rssi.csv")]
tx = [(int(r["seq"]), float(r["t_tx"])) for r in rows(f"{log}/udp_tx.csv")]
rxs = {int(r["seq"]) for r in rows(f"{log}/udp_rx.csv")}
sw_rx = [(int(r["seq"]), float(r["t_rx_s"]), float(r["latency_ms"])) for r in rows(f"{log}/swarm_follower.csv")]

ping_rep, ping_tsend = {}, {}
pat = re.compile(r"\[(\d+\.\d+)\] \d+ bytes from .*icmp_seq=(\d+) .*time=([\d.]+) ms")
pat_na = re.compile(r"\[(\d+\.\d+)\] no answer yet for icmp_seq=(\d+)")
if os.path.exists(f"{log}/ping.txt"):
    for line in open(f"{log}/ping.txt"):
        m, na = pat.search(line), pat_na.search(line)
        if m:                                   # reply: send time = reply time - rtt
            ping_rep[int(m[2])] = float(m[3])
            ping_tsend[int(m[2])] = float(m[1]) - float(m[3]) / 1e3
        elif na:                                # printed when the NEXT packet is sent (-i 0.2)
            ping_tsend.setdefault(int(na[2]), float(na[1]) - 0.2)
ping_t0 = True if ping_tsend else None
swarm_t0 = st.median([t - lat / 1e3 - seq * 0.1 for seq, t, lat in sw_rx]) if sw_rx else None

# ---- per-step stats ----
stats = []
for w in wins:
    r = {"step": w["i"], "pass": w["pass"], "d_m": w["d"]}
    rs = [v for t, v in rssi if w["t0"] <= t <= w["t1"]]
    r["rssi_n"], r["rssi_med_dbm"] = len(rs), (st.median(rs) if rs else None)
    sent = [s for s, t in tx if w["t0"] <= t <= w["t1"]]
    r["udp_sent"] = len(sent)
    r["udp_loss"] = (1 - sum(1 for s in sent if s in rxs) / len(sent)) if sent else None
    if ping_t0 is not None:
        seqs = [q for q, t in ping_tsend.items() if w["t0"] <= t <= w["t1"]]
        got = [ping_rep[q] for q in seqs if q in ping_rep]
        r["ping_sent"] = len(seqs)
        r["ping_loss"] = 1 - len(got) / len(seqs) if seqs else None
        r["rtt_p50_ms"], r["rtt_p95_ms"] = (st.median(got), pct(got, 0.95)) if got else (None, None)
    if swarm_t0 is not None:
        exp = list(range(int(math.ceil((w["t0"] - swarm_t0) / 0.1)), int((w["t1"] - swarm_t0) / 0.1)))
        got = [(s, lat) for s, t, lat in sw_rx if s in set(exp)]
        r["swarm_sent"] = len(exp)
        r["swarm_loss"] = 1 - len(got) / len(exp) if exp else None
        lats = [l for _, l in got]
        r["swarm_lat_p50_ms"], r["swarm_lat_p99_ms"] = (st.median(lats), pct(lats, 0.99)) if lats else (None, None)
    stats.append(r)

keys = sorted({k for r in stats for k in r}, key=lambda k: list(stats[0]).index(k) if k in stats[0] else 99)
with open(f"{log}/sweep_stats.csv", "w", newline="") as f:
    wr = csv.DictWriter(f, fieldnames=keys)
    wr.writeheader()
    wr.writerows(stats)

# ---- prediction lookup ----
prow = pred["rows"]
def model(d, key):
    return min(prow, key=lambda r: abs(r["d_m"] - d))[key]

print(f"\n{'step':>4} {'pass':>4} {'d_m':>6} {'rssi':>7} {'model':>7} {'udp%':>6} {'pm%':>6} {'ping%':>6} {'rtt50':>6} {'swarm%':>7} {'lat50':>6}")
f1 = lambda v, n=1: "   -  " if v is None else f"{v:.{n}f}"
for r in stats:
    print(f"{r['step']:>4} {r['pass']:>4} {r['d_m']:>6.0f} {f1(r['rssi_med_dbm']):>7} {model(r['d_m'],'rssi_dbm'):>7.1f} "
          f"{f1(None if r['udp_loss'] is None else 100*r['udp_loss']):>6} {100*model(r['d_m'],'udp_loss'):>6.1f} "
          f"{f1(None if r.get('ping_loss') is None else 100*r['ping_loss']):>6} {f1(r.get('rtt_p50_ms'),2):>6} "
          f"{f1(None if r.get('swarm_loss') is None else 100*r['swarm_loss']):>7} {f1(r.get('swarm_lat_p50_ms'),1):>6}")
print()

# ---- checks ----
rs_err = [(r["d_m"], abs(r["rssi_med_dbm"] - model(r["d_m"], "rssi_dbm"))) for r in stats if r["rssi_n"] >= 3]
worst = max(rs_err, key=lambda x: x[1]) if rs_err else (0, 1e9)
check("RSSI matches LogDistance model at every step with data (<= 0.5 dB)", worst[1] <= 0.5,
      f"worst {worst[1]:.3f} dB at {worst[0]:.0f} m, {len(rs_err)} steps")
d1p, d99p = pred["summary"]["d_udp_loss_1pct_m"], pred["summary"]["d_udp_loss_99pct_m"]
d_in, d_out = 0.9 * d1p, 1.1 * d99p
print(f"predicted cutoff {pred['summary']['d_udp_loss_50pct_m']:.1f} m ({pred['summary'].get('limited_by','')}); "
      f"'in range' = d <= {d_in:.1f} m, 'dead' = d >= {d_out:.1f} m")
near = [r for r in stats if r["d_m"] <= d_in]
check(f"loss ~0 inside range: UDP < 1 %, ping < 5 %, swarm < 5 % at d <= {d_in:.0f} m (both passes)",
      all((r["udp_loss"] or 0) < 0.01 and (r.get("ping_loss") or 0) < 0.05 and (r.get("swarm_loss") or 0) < 0.05 for r in near),
      f"{len(near)} steps")
far = [r for r in stats if r["d_m"] >= d_out]
check(f"link dead beyond cutoff: UDP loss > 95 % at d >= {d_out:.0f} m", bool(far) and all((1.0 if r["udp_loss"] is None else r["udp_loss"]) > 0.95 for r in far), f"{len(far)} steps")
# measured d50 on the forward pass (linear interpolation of UDP loss)
fwd = [r for r in stats if r["pass"] == "fwd" and r["udp_loss"] is not None]
d50 = None
for a, b in zip(fwd, fwd[1:]):
    if a["udp_loss"] < 0.5 <= b["udp_loss"]:
        d50 = a["d_m"] + (0.5 - a["udp_loss"]) / (b["udp_loss"] - a["udp_loss"]) * (b["d_m"] - a["d_m"])
        break
d50_pred = pred["summary"]["d_udp_loss_50pct_m"]
check(f"measured 50 % UDP-loss distance within 5 % of predicted {d50_pred:.1f} m",
      d50 is not None and abs(d50 - d50_pred) <= 0.05 * d50_pred, f"measured {None if d50 is None else round(d50, 1)}")
ret = [r for r in stats if r["pass"] == "ret" and r["d_m"] <= d_in]
check(f"moving back restores the link (UDP loss < 1 % at d <= {d_in:.0f} m on the return pass)",
      bool(ret) and all((1.0 if r["udp_loss"] is None else r["udp_loss"]) < 0.01 for r in ret), f"{len(ret)} steps")

# recovery: first delivery after the move that brought the agent back into range
rec = {}
back = next((w for w in wins if w["pass"] == "ret" and w["d"] <= d_in + 6 and any(
    (x["udp_loss"] is not None and x["udp_loss"] < 0.5) for x in stats if x["step"] == w["i"])), None)
if back is not None:
    tm = back["t_move"]
    tx_t = {int(r["seq"]): float(r["t_tx"]) for r in rows(f"{log}/udp_tx.csv")}
    # only packets SENT after the move count (older ones can be stale queue contents)
    first_udp = min((float(r["t_rx"]) for r in rows(f"{log}/udp_rx.csv") if tx_t.get(int(r["seq"]), 0) >= tm), default=None)
    first_ping = min((ping_tsend[q] + ping_rep[q] / 1e3 for q in ping_rep if ping_tsend[q] >= tm), default=None)
    first_sw = min((t for _, t, lat in sw_rx if t - lat / 1e3 >= tm), default=None)
    rec = {"udp_s": None if first_udp is None else round(first_udp - tm, 2),
           "ping_s": None if first_ping is None else round(first_ping - tm, 2),
           "swarm_s": None if first_sw is None else round(first_sw - tm, 2), "step_d_m": back["d"]}
    print(f"recovery after re-entering range at {back['d']:.0f} m (time to first delivered packet): {rec}")

summ = json.load(open(f"{log}/ns3/summary.json"))
check("ns-3 real-time lag within limit", summ["valid_ns3_lag"], f"max {summ['ns3_max_lag_ms']} ms")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from timehealth import rtf_check
rtf = [(float(r["t_wall_s"]), float(r["rtf"])) for r in rows(f"{log}/bridge/rtf.csv")]
rc = rtf_check(rtf, sw["t_window_start"], sw["t_window_end"], cfg["time"]["min_rtf"], cfg["time"].get("rtf_window_s", 5.0))
check(f"Gazebo RTF: {cfg['time'].get('rtf_window_s', 5.0):g}-s sliding mean >= {cfg['time']['min_rtf']} over the sweep window", rc["ok"],
      f"worst window mean {rc['worst_window_mean']:.3f}; worst single 100 ms sample {rc['worst_sample']:.2f}; "
      f"{rc['n_below']}/{rc['n']} samples below" if rc["n"] else "no samples")
win = [rc["worst_sample"]] if rc["n"] else []

json.dump({"d50_measured_m": d50, "d50_predicted_m": d50_pred, "rssi_worst_err_db": worst[1],
           "recovery_after_return": rec, "ns3_max_lag_ms": summ["ns3_max_lag_ms"], "rtf_worst_window_mean": rc["worst_window_mean"], "rtf_worst_sample": rc["worst_sample"],
           "all_pass": ok_all}, open(f"{log}/sweep_report.json", "w"), indent=1)

# ---- plots ----
try:
    sys.path.insert(0, os.path.dirname(__file__))
    from plot_sweep import make_plots
    make_plots(stats, pred, f"{log}/sweep.png")
    print(f"plot: {log}/sweep.png")
except Exception as e:  # plotting must never mask the test verdict
    print("plot skipped:", repr(e))
sys.exit(0 if ok_all else 1)
