#!/usr/bin/env python3
"""Move simulated agents with the Gazebo set_pose service (pre-PX4 test motion).

One persistent gz-transport participant is used for all calls: every new peer
(e.g. one `gz service` CLI process per call) stalls the Gazebo server for ~100 ms
and wrecks the real-time factor.

    sweep.py --world W --agent agent2 --x 1,5,20,... [--y-agent agent1] --hold S --out steps.json
Writes the commanded model positions with wall-clock end times.
"""
import argparse
import json
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node


def set_pose(node, world, name, x, y, z):
    req = Pose()
    req.name, req.position.x, req.position.y, req.position.z = name, x, y, z
    ok, rep = node.request(f"/world/{world}/set_pose", req, Pose, Boolean, 3000)
    if not (ok and rep.data):
        raise SystemExit(f"set_pose failed for {name}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", required=True)
    ap.add_argument("--agent", default="agent2")
    ap.add_argument("--x", required=True, help="comma list of x positions")
    ap.add_argument("--y-agent", default="agent1", help="agent that also moves in y (x/10) from x>=20")
    ap.add_argument("--z", type=float, default=1.0)
    ap.add_argument("--hold", type=float, default=2.0)
    ap.add_argument("--warmup", type=float, default=3.0)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    node = Node()
    # warm-up: first request pays the one-off peer-join stall, outside the measured window
    set_pose(node, a.world, a.agent, 10.0, 0.0, a.z)
    time.sleep(a.warmup)
    t_start = time.time()
    steps = []
    for x in [float(v) for v in a.x.split(",")]:
        set_pose(node, a.world, a.agent, x, 0.0, a.z)
        cmds = [{"agent": a.agent, "pos": [x, 0.0, a.z]}]
        if x >= 20 and a.y_agent:
            set_pose(node, a.world, a.y_agent, 0.0, x / 10.0, a.z)
            cmds.append({"agent": a.y_agent, "pos": [0.0, x / 10.0, a.z]})
        time.sleep(a.hold)
        t_end = time.time()
        for c in cmds:
            c["t_wall_end"] = t_end
            steps.append(c)
    json.dump({"t_window_start": t_start, "t_window_end": time.time(), "steps": steps}, open(a.out, "w"))


if __name__ == "__main__":
    main()
