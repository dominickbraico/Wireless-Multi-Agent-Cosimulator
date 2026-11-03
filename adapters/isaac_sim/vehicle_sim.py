#!/usr/bin/env python3
"""Isaac Sim scene for the cosimulator: one Pegasus multirotor per agent, each listening for its own PX4 SITL.

    /isaac-sim/python.sh vehicle_sim.py --config <experiment_run.yaml>       (started by orchestrator/sim_main.py in the simulator container)

For every agent in the YAML (vehicle type from orchestrator/vehicles.py, `isaac` entry) it spawns the Pegasus Iris at `spawn_pose` and opens a
MAVLink server on TCP 4560 + px4_instance on all interfaces: PX4 connects out to it (PX4_SIM_HOST_ADDR, orchestrator/simulators.py). Pegasus waits
for PX4's first heartbeat inside its step, so the simulation steps from the start. /logs/.isaac_ready is written after the first steps (the container health check).
Scene: a fixed box as ground (no NVIDIA asset: the container's network has no internet). Multirotors only: Pegasus has no other vehicle types.
Camera (agents with a `camera:` block): one Isaac Sim camera on the vehicle body, rendered at its configured rate (the scene is only rendered on steps where a frame
is due) and streamed to the agent container (adapters/isaac_sim/camera_stream.py, orchestrator/camera_sink.py). The scene then gets a distant light, a dome light and four coloured
posts around the origin so the image is not black or constant.
Pose feed: adapters/isaac_sim/pose_feed.py, a thread in this process sending each vehicle's position plus `antenna_offset` to ns-3 at network.pose.rate_hz.
Modelling note: Isaac Sim's world frame is taken as East-North-Up with the origin ns-3 uses, as for Gazebo (docs/CONTAINERS.md, pose feed): the vehicles are spawned at `spawn_pose`
in that frame and Pegasus reports positions in it. Nothing here checks that frame against anything else.
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
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import camera_sink
import camera_stream
import pose_feed

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

cam_agents = [a for a in cfg["agents"] if a.get("camera")]
if cam_agents:
    import numpy as np
    import omni.usd
    from pxr import UsdLux
    from isaacsim.sensors.camera import Camera
    stage = omni.usd.get_context().get_stage()
    UsdLux.DistantLight.Define(stage, "/World/sun").CreateIntensityAttr(3000.0)
    UsdLux.DomeLight.Define(stage, "/World/sky").CreateIntensityAttr(500.0)
    for i, (px, py, col) in enumerate([(6.0, 0.0, (1.0, 0.1, 0.1)), (0.0, 6.0, (0.1, 0.1, 1.0)), (-6.0, 0.0, (0.1, 1.0, 0.1)), (0.0, -6.0, (1.0, 1.0, 0.1))]):
        FixedCuboid(prim_path=f"/World/post{i}", name=f"post{i}", position=[px, py, 1.5], scale=[0.6, 0.6, 3.0], color=np.array(col))

pose = cfg["network"]["pose"]
sender = pose_feed.PoseSender((pose["listen_addr"], int(pose["port"])), pose["rate_hz"], {a["name"]: a["antenna_offset"] for a in cfg["agents"]})
drones = {}                                                   # agent name -> Pegasus vehicle

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
    drones[a["name"]] = Multirotor(f"/World/{a['name']}", ROBOTS[v["asset"]], int(a["px4_instance"]), [x, y, z],
               Rotation.from_euler("XYZ", [0.0, 0.0, yaw], degrees=True).as_quat(), config=mc)

cams = {}                                                     # agent name -> dict(camera, streamer, period, next, seq)
for a in cam_agents:
    c = a["camera"]
    cam = Camera(prim_path=f"/World/{a['name']}/body/cosim_camera", resolution=(int(c["width"]), int(c["height"])))
    cam.set_local_pose(np.array([0.2, 0.0, 0.0]), np.array([1.0, 0.0, 0.0, 0.0]), camera_axes="world")      # 0.2 m ahead of the body origin, looking along the body's x axis
    st = camera_stream.FrameStreamer((a["mgmt_ip"], camera_sink.PORT_BASE + int(a["px4_instance"])))
    cams[a["name"]] = {"camera": cam, "streamer": st, "period": 1.0 / float(c["rate_hz"]), "next": 0.0, "seq": 0, "w": int(c["width"]), "h": int(c["height"])}

world.reset()
for c in cams.values():
    c["camera"].initialize()
    c["streamer"].start()
timeline.play()
print(f"vehicle_sim: {len(cfg['agents'])} vehicle(s) spawned, waiting for PX4", flush=True)
steps = 0
sender.start()
for _ in range(10 if cams else 0):
    world.step(render=True)                                   # first renders compile shaders: do it before the vehicle is flying
while simulation_app.is_running():
    t = world.current_time
    due = [n for n, c in cams.items() if t >= c["next"]]
    world.step(render=bool(due))
    steps += 1
    for name, drone in drones.items():
        sender.update(name, drone.state.position, world.current_time)
    for n in due:
        c = cams[n]
        c["next"] = max(c["next"] + c["period"], t + 0.5 * c["period"])        # never try to catch up after a slow frame
        rgba = c["camera"].get_rgba()
        if rgba is None or getattr(rgba, "size", 0) == 0 or rgba.shape[0] != c["h"] or rgba.shape[1] != c["w"]:
            continue                                          # no image yet (the first renders after initialize())
        rgb = rgba[:, :, :3]
        if rgb.dtype != np.uint8:
            rgb = np.clip(rgb * (255.0 if rgb.max() <= 1.0 else 1.0), 0, 255).astype(np.uint8)
        c["streamer"].push(c["seq"], world.current_time, c["w"], c["h"], np.ascontiguousarray(rgb).tobytes())
        c["seq"] += 1
    if steps == 20:
        open(f"{LOGS}/.isaac_ready", "w").close()
