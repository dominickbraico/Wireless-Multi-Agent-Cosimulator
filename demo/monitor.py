#!/usr/bin/env python3
"""Live terminal dashboard for the two-drone demo (put it next to the Gazebo window in the video).

    demo/monitor.py <log_dir> [--once]
Reads files the test already writes: ns3/pose_applied.csv (Gazebo truth fed to ns-3), swarm_follower.csv
(what the follower received over the emulated Wi-Fi link) and the flight logs. Refreshes every second.
"""
import csv
import math
import os
import re
import sys
import time

log = sys.argv[1]
once = "--once" in sys.argv


def last_rows(path, n=400):
    try:
        with open(path, "rb") as f:
            f.seek(0, 2)
            size = f.tell()
            f.seek(max(0, size - 60000))
            lines = f.read().decode(errors="ignore").splitlines()
        return lines[-n:]
    except FileNotFoundError:
        return []


def last_pos():
    pos = {}
    for l in reversed(last_rows(f"{log}/ns3/pose_applied.csv")):
        p = l.split(",")
        if len(p) >= 9 and p[1] in ("0", "1") and p[1] not in pos:
            try:
                pos[p[1]] = (float(p[4]), float(p[5]), float(p[6]))
            except ValueError:
                pass
        if len(pos) == 2:
            break
    return pos


def link():
    rows = [l.split(",") for l in last_rows(f"{log}/swarm_follower.csv", 60)[1:]]
    rows = [r for r in rows if len(r) >= 4 and r[0].replace(".", "").isdigit()]
    if not rows:
        return None
    lat = [float(r[2]) for r in rows]
    t_last = float(rows[-1][0])
    return {"seq": int(rows[-1][1]), "lat": sum(lat) / len(lat), "age": time.time() - t_last}


def state(agent):
    try:
        txt = open(f"{log}/{agent}_flight.log", errors="ignore").read()
    except FileNotFoundError:
        return "waiting"
    if "Landed and disarmed" in txt:
        return "landed"
    if "leader link lost" in txt:
        return "LINK LOST -> landing"
    if "Trajectory complete" in txt:
        return "landing"
    if "Reached altitude" in txt:
        return "flying"
    if "Armed in offboard" in txt:
        return "taking off"
    return "waiting"


def render():
    pos = last_pos()
    ln = link()
    out = ["  TWO X500s, LEADER-FOLLOWER OVER AN EMULATED Wi-Fi LINK (ns-3)", "  " + "-" * 62]
    for i, (n, role) in enumerate((("agent1", "leader  "), ("agent2", "follower"))):
        p = pos.get(str(i))
        ps = f"E {p[0]:6.1f}  N {p[1]:6.1f}  Up {p[2]-0.1:5.1f} m" if p else "(no pose yet)"
        out.append(f"  {role} {n}:  {ps:34s} [{state(n)}]")
    if len(pos) == 2:
        d = math.dist(pos["0"][:2], pos["1"][:2])
        out.append(f"  separation (horizontal): {d:5.1f} m      (formation target: 3.0 m)")
    if ln:
        stale = "" if ln["age"] < 1.5 else "   <-- NO LEADER DATA"
        out.append(f"  radio link: leader messages received = {ln['seq']:5d}, latency {ln['lat']:5.2f} ms{stale}")
    else:
        out.append("  radio link: waiting for the leader's first message")
    out.append("  all inter-drone traffic crosses ns-3 (802.11a ad hoc); nothing else touches the radio")
    return "\n".join(out)


if once:
    print(render())
else:
    try:
        while True:
            sys.stdout.write("\033[H\033[2J" + render() + "\n")
            sys.stdout.flush()
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
