#!/usr/bin/env python3
"""Isaac Sim scene for the cosimulator: one Pegasus multirotor per agent, each listening for its own PX4 SITL.

    /isaac-sim/python.sh vehicle_sim.py --config <experiment_run.yaml>       (started by orchestrator/sim_main.py in the simulator container)

For every agent in the YAML (vehicle type from orchestrator/vehicles.py, `isaac` entry) it spawns the Pegasus Iris at `spawn_pose` and opens a
MAVLink server on TCP 4560 + px4_instance on all interfaces: PX4 connects out to it (PX4_SIM_HOST_ADDR, orchestrator/simulators.py). Pegasus waits
for PX4's first heartbeat inside its step, so the simulation steps from the start. /logs/.isaac_ready is written after the first steps (the container health check).
Scene: a fixed box as ground (no NVIDIA asset: the container's network has no internet). Multirotors only: Pegasus has no other vehicle types.
Modelling note: Isaac Sim's world frame is taken as East-North-Up with the origin ns-3 uses, as for Gazebo (docs/CONTAINERS.md, pose feed). Pose feed: not yet here.
"""
import argparse
import os
import sys

import yaml

ap = argparse.ArgumentParser()
ap.add_argument("--config", required=True)
args = ap.parse_args()
cfg = yaml.safe_load(open(args.config))

import vehicles                                              # orchestrator/ is on PYTHONPATH (isaac-entrypoint.sh)

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})          # must follow the import directly (Pegasus examples)

import omni.timeline
from omni.isaac.core.world import World
from scipy.spatial.transform import Rotation

# Pegasus' params.py asks NVIDIA's asset server for its root path when imported and raises if it cannot be reached. The simulator container is on an
# internal network (no internet) and this scene loads no NVIDIA asset, so the lookup is answered with an empty path (observed failure: RuntimeError
# "Could not find assets root folder"). The Iris model is a local file inside Pegasus. NVIDIA environments (Hospital, Warehouse, ...) are unavailable.
import isaacsim.storage.native as _nucleus
_nucleus.get_assets_root_path = lambda *a, **k: ""

from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

LOGS = "/logs"
timeline = omni.timeline.get_timeline_interface()
pg = PegasusInterface()
pg._world = World(**pg._world_settings)
world = pg.world
# Ground: a fixed box 400 m x 400 m x 0.2 m whose top face is z = 0. `world.scene.add_default_ground_plane()` is not used: it loads its plane from NVIDIA's
# asset server (observed: RuntimeError "Could not find assets root folder" on the offline network).
from isaacsim.core.api.objects import FixedCuboid
FixedCuboid(prim_path="/World/ground", name="ground", position=[0.0, 0.0, -0.1], scale=[400.0, 400.0, 0.2])

for a in cfg["agents"]:
    v = vehicles.for_simulator(a, "isaac")
    if vehicles.kind(a) != "multirotor":
        sys.exit(f"vehicle_sim: agent {a['name']} is a {vehicles.kind(a)}; Pegasus supports multirotors only")
    x, y, z, yaw = (float(c) for c in a["spawn_pose"])
    mc = MultirotorConfig()
    mc.backends = [PX4MavlinkBackend(PX4MavlinkBackendConfig({
        "vehicle_id": int(a["px4_instance"]),                 # added to connection_baseport: the same 4560 + instance PX4 uses
        "connection_type": "tcpin",
        "connection_ip": "0.0.0.0",                           # Pegasus' default is localhost, which another container cannot reach
        "connection_baseport": 4560,
        "px4_autolaunch": False,                              # PX4 runs in the agent container
        "px4_dir": "/tmp",
        "px4_vehicle_model": v["sim_model"],
    }))]
    Multirotor(f"/World/{a['name']}", ROBOTS[v["asset"]], int(a["px4_instance"]), [x, y, z],
               Rotation.from_euler("XYZ", [0.0, 0.0, yaw], degrees=True).as_quat(), config=mc)

world.reset()
timeline.play()
print(f"vehicle_sim: {len(cfg['agents'])} vehicle(s) spawned, waiting for PX4", flush=True)
steps = 0
while simulation_app.is_running():
    world.step(render=False)
    steps += 1
    if steps == 20:
        open(f"{LOGS}/.isaac_ready", "w").close()
