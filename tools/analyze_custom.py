#!/usr/bin/env python3
"""Checks of a `scenario.type: custom` run (docs/CUSTOM_CONTROLLERS.md): did every agent's launch finish, did only the listed
topics use the radio, was the run real-time, and how well did each radio topic get through.

    analyze_custom.py <log dir> <experiment.yaml> [--min-delivery RATIO] [--consensus [--expect-no-consensus]]

Checks that apply to any custom controller:
  finished   every <agent>.done exists and no agent reported a failed launch
  radio      tools/analyze_radio.py figures per topic and ordered pair; with --min-delivery, every pair at or above it
  validity   ns-3 lag within time.max_lag_ms; Gazebo real-time factor (time.rtf_window_s sliding mean) at least time.min_rtf from the go signal
  isolation  no local-domain (0) RTPS, XRCE-DDS or PX4 topic names on the emulated channel; radio-domain (42) RTPS present
--consensus adds the checks of the template controller (examples/template_ws), thresholds fixed before the first run:
  flight     every agent's launch log has "Landed and disarmed"
  spread     start altitudes (launch_args.start_altitude) span at least 3.0 m; over the last 5 s of the consensus phase the
             spread of the agents' measured altitudes (PX4 local position, consensus.csv measured_m) is at most 0.5 m
--expect-no-consensus is the control case (radio topic not carried): the final spread must stay above 2.0 m instead.
Writes <log dir>/custom_report.json; exit code 1 if any check failed.
"""
import argparse
import csv
import json
import os
import subprocess
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analyze_radio                                    # noqa: E402
from timehealth import rtf_check                        # noqa: E402

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
ap.add_argument("log"); ap.add_argument("yaml")
ap.add_argument("--min-delivery", type=float, default=None)
ap.add_argument("--consensus", action="store_true")
ap.add_argument("--expect-no-consensus", action="store_true")
a = ap.parse_args()
log, cfg = a.log, yaml.safe_load(open(a.yaml))
names = [x["name"] for x in cfg["agents"]]
report, failed = {"checks": []}, 0


def check(name, ok, detail=""):
    global failed
    failed += not ok
    report["checks"].append({"name": name, "ok": bool(ok), "detail": detail})
    print(f"{'PASS' if ok else 'FAIL'}  {name}" + (f"  ({detail})" if detail else ""))


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


# ---- finished ----
missing = [n for n in names if not os.path.exists(f"{log}/{n}.done")]
check("every agent's launch ended (<agent>.done)", not missing, f"missing: {missing}" if missing else "")
clog = text(f"{log}/containers.log")
bad = sorted({n for n in names if f"agent_main[{n}]: launch failed" in clog or f"agent_main[{n}]: relay failed" in clog})
check("no agent reported a failed launch or relay", not bad, f"{bad}, see containers.log" if bad else "")

# ---- radio ----
relay_logs = analyze_radio.read(log)
pairs = analyze_radio.analyze(relay_logs)
f = lambda v, n: "n/a" if v is None else f"{v:.{n}f}"
for r in sorted(pairs, key=lambda r: (r["topic"], r["tx"], r["rx"])):
    print(f"radio {r['topic']} {r['tx']}->{r['rx']}: sent={r['sent']} received={r['received']} delivery={f(r['delivery'], 3)} "
          f"latency_ms p50={f(r['latency_ms_p50'], 2)} p99={f(r['latency_ms_p99'], 2)}")
report["radio"] = pairs
if a.min_delivery is not None:
    low = [f"{r['tx']}->{r['rx']} {f(r['delivery'], 3)}" for r in pairs if r["delivery"] is None or r["delivery"] < a.min_delivery]
    want = len(names) * (len(names) - 1)
    check(f"every ordered pair delivered at least {a.min_delivery:g} of each radio topic", pairs and not low and len(pairs) >= want,
          f"{len(pairs)} pairs" + (f", below: {low}" if low else "") + ("" if len(pairs) >= want else f", expected {want}"))

# ---- validity ----
try:
    summ = json.load(open(f"{log}/ns3/summary.json"))
    check("ns-3 real-time lag within limit", summ["valid_ns3_lag"], f"max {summ['ns3_max_lag_ms']} ms")
    report["ns3_max_lag_ms"] = summ["ns3_max_lag_ms"]
except FileNotFoundError:
    check("ns-3 summary.json written (clean shutdown)", False)
rt = [(float(r["t_wall_s"]), float(r["rtf"])) for r in rows(f"{log}/bridge/rtf.csv")]
poses = [float(r["t_wall_apply_s"]) for r in rows(f"{log}/ns3/pose_applied.csv")]
if not rt or not poses:
    check("real-time-factor samples present", False, "no bridge/rtf.csv or ns3/pose_applied.csv")
else:
    win, rmin = cfg["time"].get("rtf_window_s", 5.0), cfg["time"]["min_rtf"]
    go = f"{log}/go"
    t0 = max(poses[0] + 5, os.path.getmtime(go)) if os.path.exists(go) else poses[0] + 5      # the window starts at the go signal (docs/ARCHITECTURE.md, "Run validity")
    rc = rtf_check(rt, t0, poses[-1], rmin, win)
    check(f"Gazebo RTF {win:g}-s sliding mean >= {rmin}", rc["ok"], f"worst window mean {rc['worst_window_mean']:.3f}, worst sample {rc['worst_sample']:.2f}" if rc["n"] else "no samples")
    report["rtf_worst_window_mean"] = rc["worst_window_mean"]

# ---- isolation ----
def count(pcap, flt, grep=None):
    out = subprocess.run(["tcpdump", "-Z", "root", "-r", pcap, "-nn"] + (["-A"] if grep else []) + [flt], capture_output=True, text=True).stdout
    return sum(1 for l in out.splitlines() if grep in l) if grep else len([l for l in out.splitlines() if l.strip()])


pcap = f"{log}/ns3/wifi-0-0.pcap"
if os.path.exists(pcap):
    dom0, xrce, fmu = count(pcap, "udp and portrange 7400-7650"), count(pcap, "udp and port 8888"), count(pcap, "udp", grep="fmu/")
    radio = count(pcap, "udp and portrange 17900-18150")
    check("no local-domain (0) RTPS on the emulated channel", dom0 == 0, f"{dom0} frames")
    check("no XRCE-DDS (port 8888) traffic on the emulated channel", xrce == 0, f"{xrce} frames")
    check("no PX4 topic names (fmu/...) on the emulated channel", fmu == 0, f"{fmu} frames")
    check("radio domain (42) RTPS present on the emulated channel", radio > 0, f"{radio} frames")
else:
    check("ns-3 pcap present", False)

# ---- template controller ----
if a.consensus:
    landed = [n for n in names if "Landed and disarmed" in text(f"{log}/{n}_launch.log")]
    check("every agent took off, landed and disarmed", len(landed) == len(names), f"{len(landed)} of {len(names)}")
    start = [float((x.get("launch_args") or {}).get("start_altitude", "nan")) for x in cfg["agents"]]
    check("start altitudes span at least 3.0 m", max(start) - min(start) >= 3.0, f"{min(start):g} to {max(start):g} m")
    data = {n: [(float(r["t_s"]), float(r["measured_m"])) for r in rows(f"{log}/custom/{n}/consensus.csv") if r["measured_m"]] for n in names}
    if all(data.values()):
        t_end = min(d[-1][0] for d in data.values())            # the consensus phase of the agent that finished first
        final = {n: [m for t, m in d if t_end - 5.0 <= t <= t_end] for n, d in data.items()}
        means = {n: sum(v) / len(v) for n, v in final.items() if v}
        spread = max(max(v) for v in final.values() if v) - min(min(v) for v in final.values() if v)
        detail = f"spread {spread:.2f} m; mean altitude per agent " + ", ".join(f"{n} {m:.2f}" for n, m in means.items())
        report["final_spread_m"], report["final_mean_altitude_m"] = spread, means
        if a.expect_no_consensus:
            check("control case: without the radio topic the altitudes stay apart (final spread above 2.0 m)", spread > 2.0, detail)
        else:
            check("consensus: measured altitudes within 0.5 m of each other over the last 5 s", spread <= 0.5, detail)
    else:
        check("consensus.csv written by every agent", False, str({n: len(d) for n, d in data.items()}))

json.dump(report, open(f"{log}/custom_report.json", "w"), indent=1)
sys.exit(1 if failed else 0)
