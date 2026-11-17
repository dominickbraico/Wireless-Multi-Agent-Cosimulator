#!/usr/bin/env python3
"""Entry program of an agent container: the per-agent software stack of leader_follower_fly.sh, one agent at a time.

    agent_main.py     (configured through the environment set by compose_gen.py)

COSIM_MODE=net     nothing to run: the radio interface exists, the container idles (link tests drive it with `exec`).
COSIM_MODE=flight  (the leader-follower scenario below; `scenario.type: mesh_dropout` in the YAML swaps steps 2-4 for orchestrator/scenario_mesh.py)
                   1. Micro XRCE-DDS Agent and PX4 SITL (attached to the physics simulator named in the YAML)
                   2. swarm node on the radio (leader publishes, follower subscribes); writes /logs/<agent>.px4_ready first
                   3. waits for /logs/go (written by the test driver once every agent is ready), then the flight node:
                        leader   square_trajectory; after landing, stops its swarm node (followers must then see the link go)
                        follower follower_flight (lands when the link is lost)
                   4. writes /logs/<agent>.done and idles
Log files use the names of the native scripts (<agent>_px4.log, swarm_<agent>.log, <agent>_flight.log, ...), so
tools/analyze_formation.py reads a container run unchanged.
"""
import os
import subprocess
import sys
import time

import yaml

import fleet
import simulators
import supervise
import camera_sink
import vehicles
from supervise import file_contains, spawn, wait_for

LOGS = "/logs"
PX4_BUILD = "/opt/PX4-Autopilot/build/px4_sitl_default"
XRCE = "/usr/local/bin/MicroXRCEAgent"
DDS = "/opt/cosim/config/dds"


def main():
    cfg = yaml.safe_load(open(os.environ.get("COSIM_CONFIG", "/config/experiment_run.yaml")))
    name = os.environ["COSIM_AGENT"]
    mode = os.environ.get("COSIM_MODE", "flight")
    supervise.install_handlers()
    if mode == "net":
        print(f"agent_main[{name}]: network-only mode, idling", flush=True)
        while True:
            time.sleep(60)

    agent = next(a for a in cfg["agents"] if a["name"] == name)
    leader, followers = fleet.resolve(cfg, [float(v) for v in os.environ["OFFSET"].split(",")] if os.environ.get("OFFSET") else None)
    is_leader = leader["name"] == name
    offset = dict((f["name"], o) for f, o in followers).get(name)
    inst = agent["px4_instance"]
    world = os.environ.get("COSIM_WORLD") or simulators.get(cfg)["world"](cfg)
    alt, side, speed = (os.environ.get(k, d) for k, d in (("ALT", "5.0"), ("SIDE", "10.0"), ("SPEED", "1.5")))
    sx, sy = float(agent["spawn_pose"][0]), float(agent["spawn_pose"][1])

    # PX4 and the XRCE agent must not pick up ROS's library path (they ship their own Fast DDS); the real `gz` must win on PATH.
    base = dict(os.environ, PATH="/usr/bin:" + os.environ["PATH"], GZ_CONFIG_PATH="/usr/share/gz")
    xenv = dict(base, LD_LIBRARY_PATH="/usr/local/lib", FASTRTPS_DEFAULT_PROFILES_FILE=f"{DDS}/fastdds_loopback.xml")
    penv = dict(base, LD_LIBRARY_PATH="/usr/local/lib", **simulators.get(cfg)["px4_env"](cfg, agent, world))
    wd = f"{PX4_BUILD}/instance_{inst}"
    os.makedirs(wd, exist_ok=True)

    spawn([XRCE, "udp4", "-p", "8888"], f"{LOGS}/{name}_xrce.log", env=xenv, label="xrce")
    spawn([f"{PX4_BUILD}/bin/px4", "-i", str(inst), "-d", f"{PX4_BUILD}/etc"], f"{LOGS}/{name}_px4.log", env=penv, cwd=wd, label="px4")
    px4_log = f"{LOGS}/{name}_px4.log"
    if agent.get("camera"):                              # camera frames from the simulator arrive here over the management network (orchestrator/camera_sink.py)
        spawn(["python3", "/opt/cosim/orchestrator/camera_sink.py", "--bind", agent["mgmt_ip"], "--port", str(camera_sink.PORT_BASE + inst), "--csv", f"{LOGS}/camera_{name}.csv"]
              + (["--ppm-dir", f"{LOGS}/camera_{name}_frames", "--ppm-every", str(int(agent["camera"]["save_every"]))] if agent["camera"].get("save_every") else []),
              f"{LOGS}/camera_sink_{name}.log", label="camera")
    def px4_ready():
        """PX4 prints "Ready for takeoff!" only for rotary- and fixed-wing vehicles (Commander::checkAndInformReadyForTakeoff, SITL builds), never
        for a rover, so a rover is asked directly: `commander check` answers "Preflight check: OK" when every arming check passes."""
        if vehicles.kind(agent) == "multirotor":
            return file_contains(px4_log, "Ready for takeoff")
        def px4(tool, *args):
            try:
                r = subprocess.run([f"{PX4_BUILD}/bin/px4-{tool}", "--instance", str(inst), *args], cwd=wd, env=penv, capture_output=True, text=True, timeout=10)
            except subprocess.TimeoutExpired:
                return ""
            return r.stdout + r.stderr
        if "Preflight check: OK" not in px4("commander", "check"):
            return False
        if vehicles.kind(agent) != "fixedwing":
            return True
        # A fixed-wing is ready only when PX4's altitude controller (TECS) has brought its internal altitude reference to the vehicle's altitude. At the pinned PX4
        # commit that reference starts near 0 m above mean sea level about 10 s after boot and approaches the true altitude (488 m in PX4's worlds) at
        # FW_T_CLMB_R_SP = 3 m/s, so it needs about 175 s. Taking off earlier ends in a descent into the ground at the maximum sink rate once the take-off
        # climb hands over to TECS (observed 2026-11-10, three flights; dev_docs notes). 2 m is far below that error and above the estimator's noise.
        def field(text, key):
            for line in text.splitlines():
                p = line.split()
                if len(p) == 2 and p[0] == key + ":":
                    try:
                        return float(p[1])
                    except ValueError:
                        return None
            return None
        ref, alt = field(px4("listener", "tecs_status", "1"), "altitude_reference"), field(px4("listener", "vehicle_global_position", "1"), "alt")
        return ref is not None and alt is not None and abs(ref - alt) < 2.0

    ready_s = 420 if vehicles.kind(agent) == "fixedwing" else 240       # a fixed-wing waits about 175 s for PX4's altitude reference (px4_ready)
    if not wait_for(lambda: px4_ready() or supervise.died(), ready_s, period=1.0):
        print(f"agent_main[{name}]: PX4 not ready within {ready_s} s", flush=True)
        supervise.stop_all()
    if supervise.died():
        print(f"agent_main[{name}]: process exited before PX4 was ready: {supervise.died()}", flush=True)
        supervise.stop_all()
    open(f"{LOGS}/{name}.px4_ready", "w").close()
    print(f"agent_main[{name}]: PX4 ready", flush=True)

    scenario = (cfg.get("scenario") or {}).get("type", "leader_follower")
    if scenario == "mesh_dropout":                       # a different flight; everything above (PX4, XRCE) is shared
        import scenario_mesh
        return scenario_mesh.run(cfg, agent, name, inst, (sx, sy), LOGS, DDS)
    if scenario == "rover_drive":                        # ground vehicles: PX4 stack above is shared, the drive is its own node
        import scenario_rover
        return scenario_rover.run(cfg, agent, name, inst, LOGS)
    if scenario == "custom":                             # the user's own ROS 2 launch file plus the radio relay (docs/CUSTOM_CONTROLLERS.md)
        import scenario_custom
        return scenario_custom.run(cfg, agent, name, inst, LOGS, DDS)
    if scenario != "leader_follower":
        print(f"agent_main[{name}]: unknown scenario.type '{scenario}' (leader_follower, mesh_dropout, rover_drive, custom)", flush=True)
        supervise.stop_all()

    # swarm node: the only process on the radio domain
    senv = dict(os.environ, SWARM_MODE="px4", CYCLONEDDS_URI=f"file://{DDS}/cyclonedds_agent.xml")
    sargs = ["ros2", "run", "swarm_control", "swarm_node", "--ros-args", "-p", f"role:={'leader' if is_leader else 'follower'}",
             "-p", f"agent_id:={inst + 1}", "-p", f"px4_instance:={inst}", "-p", f"spawn_xy:=[{sx},{sy}]"] + fleet.qos_args(cfg)
    if is_leader:
        sargs += ["-p", "rate_hz:=10.0"]
    else:
        sargs += ["-p", f"offset:=[{offset[0]},{offset[1]},{offset[2]}]", "-p", f"log_csv:={LOGS}/swarm_follower_{name}.csv"]
    swarm = spawn(sargs, f"{LOGS}/swarm_{name}.log", env=senv, label="swarm")

    if not wait_for(lambda: os.path.exists(f"{LOGS}/go") or supervise.died(), 900):
        print(f"agent_main[{name}]: no go signal within 900 s", flush=True)
        supervise.stop_all()

    # flight node: local DDS domain only
    lenv = dict(os.environ, ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST", ROS_DOMAIN_ID="0")
    flight_log = f"{LOGS}/{name}_flight.log"
    if is_leader:
        fargs = ["ros2", "run", "px4_offboard_control", "square_trajectory", "--ros-args", "-p", f"px4_instance:={inst}",
                 "-p", f"side:={side}", "-p", f"altitude:={alt}", "-p", f"speed:={speed}"]
    else:
        fargs = ["ros2", "run", "px4_offboard_control", "follower_flight", "--ros-args", "-p", f"px4_instance:={inst}",
                 "-p", f"altitude:={alt}", "-p", f"spawn_xy:=[{sx},{sy}]", "-p", "on_link_lost:=land", "-p", "link_timeout_s:=2.5"]
    spawn(fargs, flight_log, env=lenv, label="flight")

    landed = lambda: file_contains(flight_log, "Landed and disarmed")
    if is_leader:
        wait_for(landed, 400)
        time.sleep(2)
        supervise.kill_group(swarm)            # the link goes away: every follower must now detect it and land
        print(f"agent_main[{name}]: landed; swarm node stopped", flush=True)
    else:
        wait_for(landed, 500)
        print(f"agent_main[{name}]: landed={landed()}", flush=True)
    open(f"{LOGS}/{name}.done", "w").close()
    while True:
        time.sleep(60)


if __name__ == "__main__":
    sys.exit(main())
