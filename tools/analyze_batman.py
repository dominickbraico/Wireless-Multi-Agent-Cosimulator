#!/usr/bin/env python3
"""Checks of the batman-adv transition flight (config/experiments/batman_transition.yaml, tests/containers_batman_transition.sh).

    analyze_batman.py <log dir> <experiment.yaml> [--src agent1 --dst agent3 --relay agent2]

One drone (dst) flies out of radio range of another (src) while staying in range of a third (relay). The criteria were fixed
before the first flight (docs/EXPERIMENTS.md, "batman-adv mesh"):
  T1  src's batman-adv next hop toward dst: dst before the modelled drop of the src-dst link, relay during it for at least 5 s,
      dst again after the modelled recovery. The drop interval is where ns-3's pair_rssi.csv has src-dst below the receiver floor,
      mapped from ns-3 time to wall time with ns3/pose_applied.csv.
  T2  ping src -> dst at 5 Hz from the go signal: at least 90 % answered; total outage at most 10 s, where the outage is the sum
      of (gap - 0.2 s) over consecutive replies more than 0.5 s apart.
  T3  heartbeats (mesh/mesh_events_<agent>.csv): the longest LOST-to-UP interval between src and dst, in either log, is at most 10 s.
  T4  ns-3 lag and Gazebo real-time factor as in the other scenarios; captures on each agent's radio0 (the batman-adv interface,
      before encapsulation) hold no UDP 7400-7650 or 8888 and no "fmu/" string and do hold domain-42 RTPS; captures on mesh0
      hold only batman-adv frames among those the agent sent (source MAC = its mesh0 MAC); the mesh0 IP firewall counters are 0.
  T5  relay's mesh0 capture holds src's pings to dst arriving from src and leaving to dst, and the replies back: at least 20 frames
      in each of the four directions.
Inputs: mesh_routes/<agent>.csv (orchestrator/batman_routes.py), ping_<src>_<dst>.txt (ping -D), mesh/, ns3/, capture/<agent>_{radio0,mesh0}.pcap,
mesh_firewall_counters.txt, go. Writes batman_report.json; exit 1 if any check failed.
"""
import argparse
import csv
import json
import os
import re
import statistics
import subprocess
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "orchestrator"))
import batman_pcap                                      # noqa: E402
import containers                                       # noqa: E402
from timehealth import rtf_check                        # noqa: E402

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
ap.add_argument("log"); ap.add_argument("yaml")
ap.add_argument("--src", default="agent1"); ap.add_argument("--dst", default="agent3"); ap.add_argument("--relay", default="agent2")
a = ap.parse_args()
log, cfg = a.log, yaml.safe_load(open(a.yaml))
src, dst, relay = a.src, a.dst, a.relay
plan = containers.plan(cfg)["agents"]
ag = {x["name"]: x for x in cfg["agents"]}
report, failed = {"checks": []}, 0


def check(name, ok, detail=""):
    global failed
    failed += not ok
    report["checks"].append({"name": name, "ok": bool(ok), "detail": detail})
    print(f"{'PASS' if ok else 'FAIL'}  {name}" + (f"  ({detail})" if detail else ""))


def rows(path):
    try:
        return list(csv.DictReader(open(path)))
    except FileNotFoundError:
        return []


go = os.path.getmtime(f"{log}/go") if os.path.exists(f"{log}/go") else None

# ---- ns-3 time -> wall time, and the modelled drop of the src-dst link ----
poses = rows(f"{log}/ns3/pose_applied.csv")
offset = statistics.median(float(r["t_wall_apply_s"]) - float(r["t_ns3_s"]) for r in poses) if poses else None
below = sorted(float(r["t_s"]) for r in rows(f"{log}/ns3/pair_rssi.csv")
               if {r["tx"], r["rx"]} == {src, dst} and r["below_floor"] == "1")
intervals = []
for t in below:                                          # merge samples (0.5 s apart) into intervals
    if intervals and t - intervals[-1][1] <= 1.01:
        intervals[-1][1] = t
    else:
        intervals.append([t, t])
intervals = [iv for iv in intervals if iv[1] - iv[0] >= 2.0]
if offset is None or not intervals:
    check(f"modelled drop of the {src}-{dst} link found", False, "no pose map" if offset is None else "the link never fell below the floor for 2 s")
    drop = None
else:
    drop = max(intervals, key=lambda iv: iv[1] - iv[0])
    drop = (drop[0] + offset, drop[1] + offset)
    report["modelled_drop_wall_s"] = [round(drop[0], 2), round(drop[1], 2)]
    print(f"modelled drop of {src}-{dst}: {drop[1] - drop[0]:.1f} s ({len(intervals)} interval(s) of at least 2 s)")

# ---- T1 routes ----
route = [(float(r["t_s"]), r["next_hop"]) for r in rows(f"{log}/mesh_routes/{src}.csv") if r["originator"] == dst]
if drop and route:
    before = [nh for t, nh in route if t < drop[0] and (go is None or t >= go)]
    during = [(t, nh) for t, nh in route if drop[0] <= t <= drop[1]]
    via = [t for t, nh in during if nh == relay]
    after = [nh for t, nh in route if t > drop[1]]
    span = (max(via) - min(via) + 1.0) if via else 0.0
    ok = bool(before) and before[-1] == dst and span >= 5.0 and dst in after
    seq = []
    for t, nh in route:
        if not seq or seq[-1][1] != nh:
            seq.append((t, nh))
    detail = " -> ".join(f"{nh} at {t - (go or route[0][0]):+.0f} s" for t, nh in seq)
    check(f"T1 {src}'s next hop to {dst}: direct, then via {relay} for at least 5 s during the drop ({span:.0f} s), then direct", ok, detail)
    report["route_changes"] = [{"t_rel_s": round(t - (go or route[0][0]), 1), "next_hop": nh} for t, nh in seq]
else:
    check(f"T1 route log of {src} toward {dst}", False, f"{len(route)} rows")

# ---- T2 ping ----
ping = f"{log}/ping_{src}_{dst}.txt"
replies, last_seq = [], 0
for line in open(ping, errors="ignore") if os.path.exists(ping) else []:
    m = re.match(r"^\[(\d+\.\d+)\].*icmp_seq=(\d+).* time=", line)
    if m:
        replies.append(float(m.group(1)))
    s = re.search(r"icmp_seq=(\d+)", line)
    if s:
        last_seq = max(last_seq, int(s.group(1)))
sent = re.search(r"(\d+) packets transmitted", open(ping).read()) if os.path.exists(ping) else None
sent = int(sent.group(1)) if sent else last_seq
if replies and sent:
    replies.sort()
    gaps = [b - x for x, b in zip(replies, replies[1:])]
    outage = sum(g - 0.2 for g in gaps if g > 0.5)
    frac = len(replies) / sent
    check(f"T2 ping {src} -> {dst}: at least 90 % answered, total outage at most 10 s",
          frac >= 0.90 and outage <= 10.0, f"{len(replies)} of {sent} ({100 * frac:.1f} %), outage {outage:.1f} s, longest gap {max(gaps, default=0):.1f} s")
    report["ping"] = {"sent": sent, "answered": len(replies), "outage_s": round(outage, 2), "longest_gap_s": round(max(gaps, default=0), 2)}
else:
    check(f"T2 ping log {ping}", False, "no replies")

# ---- T3 heartbeats ----
idn = {n: int(x["px4_instance"]) + 1 for n, x in ag.items()}
longest, seen = 0.0, 0
for me, peer in ((src, dst), (dst, src)):
    lost_at = None
    for r in rows(f"{log}/mesh/mesh_events_{me}.csv"):
        if int(r["peer_id"]) != idn[peer]:
            continue
        seen += 1
        if r["event"] == "LOST":
            lost_at = float(r["t_s"])
        elif r["event"] == "UP" and lost_at is not None:
            longest = max(longest, float(r["t_s"]) - lost_at); lost_at = None
check(f"T3 heartbeats {src} <-> {dst}: longest LOST-to-UP interval at most 10 s", seen > 0 and longest <= 10.0,
      f"longest {longest:.1f} s" if seen else "no heartbeat events")
report["heartbeat_longest_lost_s"] = round(longest, 2)

# ---- T4 validity ----
try:
    summ = json.load(open(f"{log}/ns3/summary.json"))
    check("T4 ns-3 real-time lag within limit", summ["valid_ns3_lag"], f"max {summ['ns3_max_lag_ms']} ms")
except FileNotFoundError:
    check("T4 ns-3 summary.json written (clean shutdown)", False)
rt = [(float(r["t_wall_s"]), float(r["rtf"])) for r in rows(f"{log}/bridge/rtf.csv")]
if rt and poses:
    win, rmin = cfg["time"].get("rtf_window_s", 5.0), cfg["time"]["min_rtf"]
    t0 = max(float(poses[0]["t_wall_apply_s"]) + 5, go or 0)
    rc = rtf_check(rt, t0, float(poses[-1]["t_wall_apply_s"]), rmin, win)
    check(f"T4 Gazebo RTF {win:g}-s sliding mean >= {rmin}", rc["ok"], f"worst window mean {rc['worst_window_mean']:.3f}" if rc["n"] else "no samples")
else:
    check("T4 real-time-factor samples present", False)

# ---- T4 isolation ----
def count(pcap, flt, grep=None):
    out = subprocess.run(["tcpdump", "-Z", "root", "-r", pcap, "-nn"] + (["-A"] if grep else []) + [flt], capture_output=True, text=True).stdout
    return sum(1 for l in out.splitlines() if grep in l) if grep else len([l for l in out.splitlines() if l.strip()])


dom0 = xrce = fmu = radio = nonbat = 0
missing = []
for n in ag:
    r0, m0 = f"{log}/capture/{n}_radio0.pcap", f"{log}/capture/{n}_mesh0.pcap"
    if not (os.path.exists(r0) and os.path.exists(m0)):
        missing.append(n); continue
    dom0 += count(r0, "udp and portrange 7400-7650"); xrce += count(r0, "udp and port 8888"); fmu += count(r0, "udp", grep="fmu/")
    radio += count(r0, "udp and portrange 17900-18149")
    kinds, _ = batman_pcap.decode(m0, {plan[n]["mac"]})            # frames this agent sent; arrivals from the host's Docker bridge are not counted
    nonbat += sum(v for k, v in kinds.items() if k.startswith("non-batman"))
fw = open(f"{log}/mesh_firewall_counters.txt").read() if os.path.exists(f"{log}/mesh_firewall_counters.txt") else ""
ipm = sum(int(x) for x in re.findall(r"packets (\d+)", fw)) if fw else None
check("T4 isolation: no local-domain RTPS, XRCE or PX4 topic names on radio0; domain-42 RTPS present",
      not missing and dom0 == 0 and xrce == 0 and fmu == 0 and radio > 0, f"domain 0 {dom0}, XRCE {xrce}, fmu {fmu}, domain 42 {radio} frames" + (f"; no captures for {missing}" if missing else ""))
check("T4 only batman-adv frames on mesh0; mesh0 IP firewall counters 0", not missing and nonbat == 0 and ipm == 0,
      f"non-batman frames {nonbat}, mesh0 IP drops {ipm if ipm is not None else 'no counters'}")

# ---- T5 relay ----
cap = f"{log}/capture/{relay}_mesh0.pcap"
if os.path.exists(cap):
    _, flows = batman_pcap.decode(cap)
    ms, md, mr = plan[src]["mac"], plan[dst]["mac"], plan[relay]["mac"]
    ips, ipd = ag[src]["radio_ip"], ag[dst]["radio_ip"]
    n = lambda f, t, x, y: sum(c for (a1, b1, x1, y1, _), c in flows.items() if (a1, b1, x1, y1) == (f, t, x, y))
    legs = {"in": n(ms, mr, ips, ipd), "out": n(mr, md, ips, ipd), "back-in": n(md, mr, ipd, ips), "back-out": n(mr, ms, ipd, ips)}
    check(f"T5 {relay} relays {src}'s pings to {dst} and the replies (at least 20 frames each way)", min(legs.values()) >= 20,
          ", ".join(f"{k} {v}" for k, v in legs.items()))
    report["relay_frames"] = legs
else:
    check(f"T5 capture of {relay}'s mesh0", False, cap)

json.dump(report, open(f"{log}/batman_report.json", "w"), indent=1)
sys.exit(1 if failed else 0)
