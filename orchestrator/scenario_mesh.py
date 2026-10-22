#!/usr/bin/env python3
"""The mesh-dropout scenario, as run by one agent container after PX4 is ready (called from agent_main.py).

Selected by `scenario.type: mesh_dropout` in the experiment YAML; the leader-follower flight is the default and is not touched by this file.
Every agent runs a mesh node (all-to-all heartbeat on the radio, logs which neighbours it hears). Then, once /logs/go exists:
  mode roam (default): every agent runs mesh_flight role=roam. After take-off each touches /logs/<agent>.airborne; when all have, the first agent's supervisor
    touches /logs/mesh_start and every agent flies its own seeded random path on that shared timeline, then lands at its spawn point at the planned end time.
  mode excursion:
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


def fleet_first(cfg):
    return cfg["agents"][0]["name"]


def run(cfg, agent, name, inst, spawn_xy, logs, dds_dir):
    s = mesh_plan.settings(cfg)
    plan = mesh_plan.plan(cfg)
    sx, sy = spawn_xy
    roam = plan["mode"] == "roam"
    wanderer = (not roam) and name in plan["wanderers"]
    if wanderer or roam:                                 # every flight node of the mode needs the plan file; identical content, atomic replace
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
             "-p", f"role:={'roam' if roam else 'wander' if wanderer else 'anchor'}", "-p", f"agent_name:={name}", "-p", f"plan_file:={logs}/mesh_plan.json"]
    wanderers = list(plan.get("wanderers", {}))
    if roam:
        fargs += ["-p", f"airborne_file:={logs}/{name}.airborne", "-p", f"start_file:={logs}/mesh_start"]
    elif not wanderer:
        longest = max(w["expected_flight_s"] for w in plan["wanderers"].values())
        fargs += ["-p", f"land_when_file:={logs}/mesh_land_anchors", "-p", f"max_hold_s:={longest + 240.0}"]
    spawn(fargs, flight_log, env=lenv, label="flight")
    if roam and name == fleet_first(cfg):
        # the shared start: once every agent has reached its altitude (each touches <agent>.airborne), one second later every flight node begins its plan
        names = [a["name"] for a in cfg["agents"]]
        if wait_for(lambda: all(os.path.exists(f"{logs}/{n}.airborne") for n in names), 600):
            time.sleep(1.0)
            open(f"{logs}/mesh_start", "w").close()
            print(f"agent_main[{name}]: every agent is airborne, start signal given", flush=True)
        else:
            print(f"agent_main[{name}]: not every agent reached its altitude within 600 s; no start signal", flush=True)

    landed = lambda: file_contains(flight_log, "Landed and disarmed")
    if not roam and not wanderer:
        # land once every wanderer is down: the supervisors share /logs, so <wanderer>.done is the signal
        wait_for(lambda: all(os.path.exists(f"{logs}/{w}.done") for w in wanderers), 900)
        open(f"{logs}/mesh_land_anchors", "w").close()
        print(f"agent_main[{name}]: wanderers are down, anchor lands", flush=True)
    wait_for(landed, 500)
    print(f"agent_main[{name}]: landed={landed()}", flush=True)
    open(f"{logs}/{name}.done", "w").close()
    while True:
        time.sleep(60)
