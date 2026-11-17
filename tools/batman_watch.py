#!/usr/bin/env python3
"""Live view of one batman-adv route during a run: how agent SRC reaches agent DST, and whether its pings get through.

    batman_watch.py <log dir> [--src agent1] [--dst agent3]

Follows <log dir>/mesh_routes/<src>.csv (orchestrator/batman_routes.py, one row per originator per second) and
<log dir>/ping_<src>_<dst>.txt (ping -D). Prints one line per second, and a banner whenever the route changes. Used by
demo/run_container_demo.sh in its own terminal window; stops with Ctrl+C.
"""
import argparse
import csv
import os
import re
import time

RESET, BOLD, GREEN, YELLOW, RED = "\033[0m", "\033[1m", "\033[32m", "\033[33m", "\033[31m"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--src", default="agent1")
    ap.add_argument("--dst", default="agent3")
    a = ap.parse_args()
    routes, ping = f"{a.log}/mesh_routes/{a.src}.csv", f"{a.log}/ping_{a.src}_{a.dst}.txt"
    print(f"{BOLD}batman-adv route {a.src} -> {a.dst}{RESET}   (waiting for {routes})", flush=True)
    while not os.path.exists(routes):
        time.sleep(0.5)
    f = open(routes)
    reader = csv.reader(f)
    next(reader, None)
    last, pos = None, 0
    while True:
        line = f.readline()
        if not line:
            time.sleep(0.2)
            continue
        r = next(csv.reader([line]))
        if len(r) < 3 or r[1] != a.dst:
            continue
        nh = r[2]
        state = ("unreachable", RED) if nh == "none" else (("DIRECT", GREEN) if nh == a.dst else (f"via {nh} (2 hops)", YELLOW))
        replies = lost = 0
        if os.path.exists(ping):                          # pings since the previous line
            with open(ping, errors="ignore") as p:
                p.seek(pos)
                chunk = p.read()
                pos = p.tell()
            replies = len(re.findall(r" time=", chunk))
            lost = len(re.findall(r"no answer|Unreachable", chunk))
        if state[0] != last:
            print(f"\n{BOLD}{state[1]}>>> {a.src} now reaches {a.dst} {state[0]}{RESET}\n", flush=True)
            last = state[0]
        stamp = time.strftime("%H:%M:%S", time.localtime(float(r[0])))
        print(f"{stamp}  {a.src} -> {a.dst}: {state[1]}{state[0]:<20}{RESET} tq {r[4] or '-':>3}   pings answered this second: {replies}", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
