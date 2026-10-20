#!/usr/bin/env python3
"""Registry of physics simulators the cosimulator can run in the `simulator` container.

The rest of the system depends on a physics simulator through exactly two interfaces, and this module is where both are
described for each simulator type (`simulator.type` in the experiment YAML):

  1. Vehicle link: how each agent's PX4 SITL talks to the simulator.      -> px4_env()
  2. Pose feed:    how ground-truth positions reach ns-3.                 -> processes()

Everything else (ns-3, the radio, ROS 2, the swarm protocol) is simulator-independent. Adding a simulator = one entry here
plus a Dockerfile; see docs/CONTAINERS.md, "Adding a physics simulator (Isaac Sim)".
"""
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
    return {
        "PX4_GZ_STANDALONE": "1",                       # PX4 attaches to a running Gazebo instead of starting its own
        "PX4_SYS_AUTOSTART": "4001",
        "PX4_SIM_MODEL": "gz_x500",
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


SIMULATORS = {
    "gazebo": {
        "image": "cosim-sim-gazebo",
        "dockerfile": "docker/sim-gazebo.Dockerfile",
        "world": _gazebo_world,
        "service": _gazebo_service,
        "px4_env": _gazebo_px4_env,
        "processes": _gazebo_processes,
    },
}


def get(cfg):
    t = cfg["simulator"]["type"]
    if t not in SIMULATORS:
        raise SystemExit(f"simulator.type '{t}' has no container driver. Available: {', '.join(sorted(SIMULATORS))}. "
                         "See docs/CONTAINERS.md, 'Adding a physics simulator (Isaac Sim)'.")
    return SIMULATORS[t]
