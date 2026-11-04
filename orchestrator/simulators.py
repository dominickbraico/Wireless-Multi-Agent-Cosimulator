#!/usr/bin/env python3
"""Registry of physics simulators the cosimulator can run in the `simulator` container.

The rest of the system depends on a physics simulator through exactly two interfaces, and this module is where both are
described for each simulator type (`simulator.type` in the experiment YAML):

  1. Vehicle link: how each agent's PX4 SITL talks to the simulator.      -> px4_env()
  2. Pose feed:    how ground-truth positions reach ns-3.                 -> processes()

Everything else (ns-3, the radio, ROS 2, the swarm protocol) is simulator-independent. Adding a simulator = one entry here
plus a Dockerfile; see docs/CONTAINERS.md, "Adding a physics simulator (Isaac Sim)".
"""
import os
import subprocess

import vehicles
from containers import plan as container_plan


def _gazebo_world(cfg, override=None):
    return override or cfg["simulator"].get("px4_world_name", "px4_flat")


def _gazebo_service(cfg, world):
    p = container_plan(cfg)
    return {
        "environment": {
            "GZ_PARTITION": cfg["simulator"]["gz_partition"],
            "GZ_IP": p["mgmt"]["sim_ip"],
            "COSIM_WORLD": world,
        },
        # the one NET_ADMIN use: a multicast route for gz-transport discovery on the (gateway-less) management network
        "cap_add": ["NET_ADMIN"],
        "healthcheck_cmd": f"gz topic -l 2>/dev/null | grep -q /world/{world}/stats",
    }


def _gazebo_px4_env(cfg, agent, world):
    x, y, z, yaw = agent["spawn_pose"]
    p = container_plan(cfg)
    v = vehicles.for_simulator(agent, "gazebo")         # airframe and model of this agent's vehicle type (default x500)
    return {
        "PX4_GZ_STANDALONE": "1",                       # PX4 attaches to a running Gazebo instead of starting its own
        "PX4_SYS_AUTOSTART": v["autostart"],
        "PX4_SIM_MODEL": v["sim_model"],
        "PX4_GZ_WORLD": world,
        "GZ_PARTITION": cfg["simulator"]["gz_partition"],
        "GZ_IP": p["agents"][agent["name"]]["mgmt_ip"],
        "PX4_GZ_MODEL_POSE": f"{x},{y},{z},0,0,{yaw}",
    }


def _gazebo_processes(cfg, world, config_path, log_dir):
    """(name, argv) pairs the simulator container runs and supervises."""
    return [
        ("gz", ["/usr/bin/gz", "sim", "-s", "-r", "--headless-rendering", f"/opt/cosim/worlds/{world}.sdf"]),
        ("pose_bridge", ["python3", "/opt/cosim/adapters/gazebo/pose_bridge.py", "--config", config_path,
                         "--log-dir", f"{log_dir}/bridge", "--world", world, "--px4-models"]),
    ]


# ---- Isaac Sim (spike, experimental): Isaac Sim 5.1.0 + Pegasus Simulator v5.1.0 in one GPU container; each agent's PX4 connects to it over the
# simulator MAVLink interface (TCP 4560 + px4_instance on the management network). Pegasus supports multirotors only. docs/CONTAINERS.md, "Isaac Sim".
ISAAC_EULA_VAR = "COSIM_ACCEPT_ISAAC_EULA"


def _docker_runtimes():
    """Names of the container runtimes of the Docker engine this host would use (`docker info`)."""
    try:
        r = subprocess.run(["docker", "info", "--format", "{{range $k, $v := .Runtimes}}{{$k}} {{end}}"], capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.TimeoutExpired):
        return []
    return r.stdout.split() if r.returncode == 0 else []


def _isaac_world(cfg, override=None):
    return override or cfg["simulator"].get("world_name", "flat_plane")


def _isaac_service(cfg, world):
    if "nvidia" not in _docker_runtimes():
        raise SystemExit("simulator.type isaac needs a GPU: the Docker engine has no 'nvidia' runtime. Install the NVIDIA Container Toolkit "
                         "(https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html), register it with "
                         "`sudo nvidia-ctk runtime configure --runtime=docker`, restart Docker, and check `docker context ls` points at that engine.")
    if os.environ.get(ISAAC_EULA_VAR) != "Y":
        raise SystemExit(f"simulator.type isaac runs NVIDIA Isaac Sim, which needs you to accept the NVIDIA Omniverse licence agreement. "
                         f"Read it, then set {ISAAC_EULA_VAR}=Y in the environment of this command. It is not set for you.")
    return {
        "environment": {"ACCEPT_EULA": "Y", "PRIVACY_CONSENT": os.environ.get("COSIM_ISAAC_PRIVACY_CONSENT", "N"), "COSIM_WORLD": world},
        "cap_add": [],
        "gpu": True,
        "healthcheck_cmd": "test -f /logs/.isaac_ready",
        "healthcheck_retries": 150,                      # first start compiles shaders: about 160 s measured on an RTX 5060 Ti
    }


def _isaac_px4_env(cfg, agent, world):
    v = vehicles.for_simulator(agent, "isaac")
    return {
        "PX4_SIM_MODEL": v["sim_model"],                 # resolves to airframe 10016_none_iris; no PX4_GZ_* variable, so the Gazebo bridge cannot start
        "PX4_SIM_HOST_ADDR": container_plan(cfg)["mgmt"]["sim_ip"],        # PX4 connects out to TCP 4560 + px4_instance there
    }


def _isaac_processes(cfg, world, config_path, log_dir):
    return [("isaac", ["/isaac-sim/python.sh", "/opt/cosim/adapters/isaac_sim/vehicle_sim.py", "--config", config_path])]


SIMULATORS = {
    "gazebo": {
        "image": "cosim-sim-gazebo",
        "dockerfile": "docker/sim-gazebo.Dockerfile",
        "world": _gazebo_world,
        "service": _gazebo_service,
        "px4_env": _gazebo_px4_env,
        "processes": _gazebo_processes,
    },
    "isaac": {
        "image": "cosim-sim-isaac",
        "dockerfile": "docker/sim-isaac.Dockerfile",
        "world": _isaac_world,
        "service": _isaac_service,
        "px4_env": _isaac_px4_env,
        "processes": _isaac_processes,
    },
}


def get(cfg):
    t = cfg["simulator"]["type"]
    if t not in SIMULATORS:
        raise SystemExit(f"simulator.type '{t}' has no container driver. Available: {', '.join(sorted(SIMULATORS))}. "
                         "See docs/CONTAINERS.md, 'Adding a physics simulator (Isaac Sim)'.")
    return SIMULATORS[t]
