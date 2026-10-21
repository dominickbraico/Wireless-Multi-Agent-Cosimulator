#!/usr/bin/env python3
"""The mesh-dropout scenario, as run by one agent container after PX4 is ready (called from agent_main.py).

Selected by `scenario.type: mesh_dropout` in the experiment YAML; the leader-follower flight is the default and is not touched by this file.
Every agent runs a mesh node (all-to-all heartbeat on the radio, logs which neighbours it hears). Then, once /logs/go exists:
    wanderer  mesh_flight role=wander: the seeded random flight of orchestrator/mesh_plan.py, ending at its spawn point and landing
    anchor    mesh_flight role=anchor: hover at the spawn point; lands once every wanderer has landed (/logs/<wanderer>.done)
Files in /logs: mesh_plan.json (the plan actually flown, written by each wanderer), mesh/mesh_rx_<agent>.csv and mesh/mesh_events_<agent>.csv
(from the mesh node), mesh_<agent>.log, <agent>_flight.log, <agent>.done. See docs/MESH_DROPOUT.md.
"""
import json
import os
import time

import fleet
import mesh_plan
import supervise
from supervise import file_contains, spawn, wait_for


def run(cfg, agent, name, inst, spawn_xy, logs, dds_dir):
    s = mesh_plan.settings(cfg)
    plan = mesh_plan.plan(cfg)
    sx, sy = spawn_xy
    wanderer = name in plan["wanderers"]
    if wanderer:
        tmp = f"{logs}/.mesh_plan.{name}.tmp"
        json.dump(plan, open(tmp, "w"), indent=1)
        os.replace(tmp, f"{logs}/mesh_plan.json")       # every wanderer writes the same content; replace is atomic

    # mesh node: the only process of this agent on the radio domain
    menv = dict(os.environ, CYCLONEDDS_URI=f"file://{dds_dir}/cyclonedds_mesh.xml")
    margs = ["ros2", "run", "mesh_control", "mesh_node", "--ros-args", "-p", f"agent_id:={inst + 1}", "-p", f"agent_name:={name}",
             "-p", f"px4_instance:={inst}", "-p", f"spawn_xy:=[{sx},{sy}]", "-p", f"log_dir:={logs}/mesh",
             "-p", f"heartbeat_hz:={float(s['heartbeat_hz'])}", "-p", f"link_timeout_s:={float(s['link_timeout_s'])}"] + fleet.qos_args(cfg, "peer_state")
    spawn(margs, f"{logs}/mesh_{name}.log", env=menv, label="mesh")

    if not wait_for(lambda: os.path.exists(f"{logs}/go") or supervise.died(), 900):
        print(f"agent_main[{name}]: no go signal within 900 s", flush=True)
        supervise.stop_all()

    lenv = dict(os.environ, ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST", ROS_DOMAIN_ID="0")
    flight_log = f"{logs}/{name}_flight.log"
    fargs = ["ros2", "run", "mesh_control", "mesh_flight", "--ros-args", "-p", f"px4_instance:={inst}",
             "-p", f"altitude:={plan['altitude_m'][name]}", "-p", f"speed:={float(s['speed_mps'])}", "-p", f"spawn_xy:=[{sx},{sy}]",
             "-p", f"role:={'wander' if wanderer else 'anchor'}", "-p", f"agent_name:={name}", "-p", f"plan_file:={logs}/mesh_plan.json"]
    wanderers = list(plan["wanderers"])
    if not wanderer:
        longest = max(w["expected_flight_s"] for w in plan["wanderers"].values())
        fargs += ["-p", f"land_when_file:={logs}/mesh_land_anchors", "-p", f"max_hold_s:={longest + 240.0}"]
        # wait for the wanderers' plan file too: an anchor may start its flight node before a wanderer wrote it (it does not read it)
    spawn(fargs, flight_log, env=lenv, label="flight")

    landed = lambda: file_contains(flight_log, "Landed and disarmed")
    if not wanderer:
        # land once every wanderer is down: the supervisors share /logs, so <wanderer>.done is the signal
        wait_for(lambda: all(os.path.exists(f"{logs}/{w}.done") for w in wanderers), 900)
        open(f"{logs}/mesh_land_anchors", "w").close()
        print(f"agent_main[{name}]: wanderers are down, anchor lands", flush=True)
    wait_for(landed, 500)
    print(f"agent_main[{name}]: landed={landed()}", flush=True)
    open(f"{logs}/{name}.done", "w").close()
    while True:
        time.sleep(60)
