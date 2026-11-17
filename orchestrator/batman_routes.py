#!/usr/bin/env python3
"""Route log of one agent's batman-adv: once per second, which neighbour it sends through to reach every other agent.

Started by agent_main.py in every agent when `network.mesh.routing` is `batman_adv` (docs/CONTAINERS.md). Reads the
originator table (`batctl meshif radio0 originators -H`) and keeps the best route per originator, the line batctl marks
with `*`. Originators are batman-adv's addresses for the other agents: their `mesh0` MACs (containers.plan "mac").

    batman_routes.py --config <yaml> --name <agent> --csv <path> [--period 1.0]

CSV columns:  t_s (wall clock), originator, next_hop, hops_hint, tq, last_seen_s
    originator, next_hop   agent names (the MAC when it is no agent of the YAML)
    hops_hint              1 when the next hop is the originator itself (direct), 2 or more otherwise. batman-adv knows only the
                           next hop, so ">= 2" is all this column can say; `batctl tr` gives the full path.
    tq                     batman IV transmit quality, 0-255, of that route
An originator that disappears from the table (its route timed out) gets a row with next_hop "none".
batctl output format, batman-adv 2024.0 (BATMAN_IV):  [*] <originator> <last-seen>s (<tq>) <next hop> [<interface>]
"""
import argparse
import csv
import re
import subprocess
import sys
import time

import yaml

import containers

LINE = re.compile(r"^\s*(\*)?\s*([0-9a-f:]{17})\s+([0-9.]+)s\s+\(\s*(\d+)\)\s+([0-9a-f:]{17})\s+\[\s*(\S+)\]")


def best_routes(text):
    """-> {originator MAC: (next hop MAC, tq, last seen s)} from `batctl originators -H` output, best route only."""
    out = {}
    for line in text.splitlines():
        m = LINE.match(line)
        if m and m.group(1):
            out[m.group(2)] = (m.group(5), int(m.group(4)), float(m.group(3)))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--config", required=True)
    ap.add_argument("--name", required=True)
    ap.add_argument("--csv", required=True)
    ap.add_argument("--period", type=float, default=1.0)
    a = ap.parse_args()
    cfg = yaml.safe_load(open(a.config))
    agent_of = {p["mac"]: n for n, p in containers.plan(cfg)["agents"].items()}
    w = csv.writer(open(a.csv, "w", buffering=1))
    w.writerow(["t_s", "originator", "next_hop", "hops_hint", "tq", "last_seen_s"])
    known = set()
    while True:
        t = time.time()
        try:
            r = subprocess.run(["batctl", "meshif", "radio0", "originators", "-H"], capture_output=True, text=True, timeout=5)
            routes = best_routes(r.stdout) if r.returncode == 0 else {}
        except subprocess.TimeoutExpired:
            routes = {}
        for orig, (nh, tq, seen) in sorted(routes.items()):
            o, n = agent_of.get(orig, orig), agent_of.get(nh, nh)
            w.writerow([f"{t:.3f}", o, n, 1 if nh == orig else 2, tq, seen])
        for gone in known - set(routes):
            w.writerow([f"{t:.3f}", agent_of.get(gone, gone), "none", "", "", ""])
        known = set(routes)
        time.sleep(max(0.0, a.period - (time.time() - t)))


if __name__ == "__main__":
    sys.exit(main())
