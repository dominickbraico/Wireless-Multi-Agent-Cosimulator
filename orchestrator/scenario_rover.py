#!/usr/bin/env python3
"""The rover-drive scenario, as run by one agent container after PX4 is ready (called from agent_main.py).

Selected by `scenario.type: rover_drive`. It checks one thing: that a ground vehicle's PX4 stack runs in the simulator and drives where it
is told. Every agent must be a rover (`agents[].vehicle: r1_rover`); each one drives a square of `scenario.rover_drive.side_m` metres at
`speed_mps` from its spawn point once /logs/go exists, then disarms. No node runs on the radio domain: the radio carries no swarm
traffic in this scenario, and ns-3 only receives the positions. Files in /logs: <agent>_flight.log, rover/rover_<agent>.csv (PX4's local
position while driving), <agent>.done. See docs/EXPERIMENTS.md, "Rover drive test".
"""
import os
import time

import supervise
import vehicles
from supervise import file_contains, spawn, wait_for

DEFAULTS = {"side_m": 6.0, "speed_mps": 1.0, "acceptance_radius_m": 0.5}


def settings(cfg):
    given = ((cfg.get("scenario") or {}).get("rover_drive")) or {}
    unknown = set(given) - set(DEFAULTS)
    if unknown:
        raise SystemExit(f"scenario.rover_drive: unknown keys {sorted(unknown)} (known: {sorted(DEFAULTS)})")
    return {**DEFAULTS, **given}


def run(cfg, agent, name, inst, logs):
    if vehicles.kind(agent) != "rover":
        raise SystemExit(f"scenario rover_drive: agent {name} is a {vehicles.kind(agent)}; every agent must be a rover")
    s = settings(cfg)
    if not wait_for(lambda: os.path.exists(f"{logs}/go") or supervise.died(), 900):
        print(f"agent_main[{name}]: no go signal within 900 s", flush=True)
        supervise.stop_all()
    lenv = dict(os.environ, ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST", ROS_DOMAIN_ID="0")
    flight_log = f"{logs}/{name}_flight.log"
    fargs = ["ros2", "run", "px4_offboard_control", "rover_drive", "--ros-args", "-p", f"px4_instance:={inst}",
             "-p", f"side:={float(s['side_m'])}", "-p", f"speed:={float(s['speed_mps'])}",
             "-p", f"acceptance_radius:={float(s['acceptance_radius_m'])}", "-p", f"log_csv:={logs}/rover/rover_{name}.csv"]
    spawn(fargs, flight_log, env=lenv, label="flight")
    done = lambda: file_contains(flight_log, "Route complete and disarmed")
    wait_for(done, 300)
    print(f"agent_main[{name}]: route complete={done()}", flush=True)
    open(f"{logs}/{name}.done", "w").close()
    while True:
        time.sleep(60)
