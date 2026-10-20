#!/usr/bin/env python3
"""Generate the Docker Compose project for an experiment from its YAML.

    compose_gen.py --config <experiment.yaml> --out <log dir> [--mode flight|net] [--firewall|--no-firewall] [--world W]

Writes <out>/compose/docker-compose.yml and <out>/compose/experiment_run.yaml (the YAML the containers read: a copy with
the container-specific values filled in). <out> itself is mounted at /logs in every container.

Services (one container each):
  simulator   physics simulator + its pose adapter                     (flight mode only)
  ns3         the network emulator, bridged to every agent
  <agent>...  one container per agent: PX4 SITL, XRCE agent, ROS 2 nodes (flight) or just a radio interface (net)

Modes:
  flight  full stack: simulator + PX4 + ROS 2 swarm/flight nodes. Radio firewall on unless --no-firewall.
  net     network only: no simulator, no PX4. Agents are bare radio endpoints for ping/UDP/ROS 2 link tests.
"""
import argparse
import copy
import os

import yaml

import containers
import simulators
import technologies

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
NS3_IMAGE, AGENT_IMAGE = "cosim-ns3", "cosim-agent"
yaml.SafeDumper.ignore_aliases = lambda self, data: True       # repeat shared values instead of emitting YAML anchors
IPV6_OFF = {"net.ipv6.conf.all.disable_ipv6": 1, "net.ipv6.conf.default.disable_ipv6": 1}


def run_config(cfg, plan, mode, firewall, world):
    """The experiment YAML as the containers see it."""
    c = copy.deepcopy(cfg)
    c.setdefault("network", {}).setdefault("radio_firewall", {"allow_udp_ports": [17900, 18149]})
    c["network"]["radio_firewall"]["enabled"] = bool(firewall)
    if "pose" in c["network"]:
        c["network"]["pose"]["listen_addr"] = plan["mgmt"]["ns3_ip"]        # ns-3 binds here, the pose adapter sends here
    if world:
        c["simulator"]["px4_world_name"] = world
    return c


def generate(cfg, out, mode="flight", firewall=None, world=None, env=None):
    env = os.environ if env is None else env
    if firewall is None:
        firewall = mode == "flight"
    plan = containers.plan(cfg)
    tech_name, tech = technologies.resolve(cfg)
    sim = simulators.get(cfg)
    world = sim["world"](cfg, world)

    cdir = os.path.join(os.path.abspath(out), "compose")
    os.makedirs(cdir, exist_ok=True)
    rcfg = run_config(cfg, plan, mode, firewall, world)
    with open(os.path.join(cdir, "experiment_run.yaml"), "w") as f:
        yaml.safe_dump(rcfg, f, sort_keys=False)

    uid, gid = str(os.getuid()), str(os.getgid())
    common_vol = [f"{cdir}:/config:ro", f"{os.path.abspath(out)}:/logs"]
    mgmt_net = plan["mgmt"]["network"]

    networks = {mgmt_net: {"driver": "bridge", "internal": True,
                           "ipam": {"config": [{"subnet": plan["mgmt"]["subnet"], "gateway": plan["mgmt"]["gateway"]}]}}}
    for name, p in plan["agents"].items():
        networks[p["link_net"]] = {"driver": "bridge", "internal": True,
                                   "ipam": {"config": [{"subnet": p["link_subnet"], "gateway": p["link_gateway"]}]}}

    services = {}
    if mode == "flight":
        s = sim["service"](cfg, world)
        services["simulator"] = {
            "image": f"{sim['image']}:latest",
            "build": {"context": ROOT, "dockerfile": sim["dockerfile"]},
            "networks": {mgmt_net: {"ipv4_address": plan["mgmt"]["sim_ip"]}},
            "cap_add": s["cap_add"],
            "environment": {**s["environment"], "COSIM_CONFIG": "/config/experiment_run.yaml"},
            "volumes": common_vol + [f"{ROOT}/worlds:/opt/cosim/worlds:ro"],
            "healthcheck": {"test": ["CMD-SHELL", s["healthcheck_cmd"]], "interval": "3s", "timeout": "5s", "retries": 60},
            "stop_grace_period": "10s",
        }

    ns3_nets = {mgmt_net: {"ipv4_address": plan["mgmt"]["ns3_ip"]}}
    for name, p in plan["agents"].items():
        ns3_nets[p["link_net"]] = {"ipv4_address": p["ns3_link_ip"]}
    services["ns3"] = {
        "image": f"{NS3_IMAGE}:latest",
        "build": {"context": ROOT, "dockerfile": "docker/ns3.Dockerfile"},
        "networks": ns3_nets,
        "cap_add": ["NET_ADMIN"],
        "devices": ["/dev/net/tun:/dev/net/tun"],
        "sysctls": IPV6_OFF,
        "environment": {"COSIM_SCENARIO": tech["scenario"], "COSIM_CONFIG": "/config/experiment_run.yaml",
                        "NS3_DURATION": env.get("NS3_DURATION", "3600")},
        "volumes": common_vol,
        "healthcheck": {"test": ["CMD-SHELL", "test -f /logs/.ns3_ready"], "interval": "2s", "timeout": "3s", "retries": 60},
        "stop_grace_period": "20s",       # ns-3 writes summary.json on SIGTERM
    }

    deps = {"ns3": {"condition": "service_healthy"}}
    if mode == "flight":
        deps["simulator"] = {"condition": "service_healthy"}
    for a in cfg["agents"]:
        n, p = a["name"], plan["agents"][a["name"]]
        ready = f"test -f /logs/{n}.px4_ready" if mode == "flight" else "ip -br addr show radio0 | grep -q UP"
        agent_env = {"COSIM_AGENT": n, "COSIM_MODE": mode, "COSIM_CONFIG": "/config/experiment_run.yaml",
                     "HOST_UID": uid, "HOST_GID": gid}
        for k in ("ALT", "SIDE", "SPEED", "OFFSET"):
            if env.get(k):
                agent_env[k] = env[k]
        services[n] = {
            "image": f"{AGENT_IMAGE}:latest",
            "build": {"context": ROOT, "dockerfile": "docker/agent.Dockerfile"},
            "hostname": n,
            "networks": {mgmt_net: {"ipv4_address": p["mgmt_ip"]}, p["link_net"]: {"ipv4_address": p["agent_link_ip"]}},
            "cap_add": ["NET_ADMIN"],
            "sysctls": IPV6_OFF,
            "depends_on": deps,
            "environment": agent_env,
            "volumes": common_vol,
            "healthcheck": {"test": ["CMD-SHELL", ready], "interval": "3s", "timeout": "3s", "retries": 100},
            "stop_grace_period": "10s",
        }

    compose = {"name": f"cosim-{cfg['experiment']['name']}".replace("_", "-").lower(),
               "services": services, "networks": networks}
    path = os.path.join(cdir, "docker-compose.yml")
    with open(path, "w") as f:
        f.write(f"# Generated by orchestrator/compose_gen.py from the experiment YAML ({tech_name} / {cfg['simulator']['type']}, "
                f"mode {mode}). Do not edit; change the YAML and regenerate.\n")
        yaml.safe_dump(compose, f, sort_keys=False)
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--config", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--mode", choices=["flight", "net"], default="flight")
    fw = ap.add_mutually_exclusive_group()
    fw.add_argument("--firewall", dest="firewall", action="store_true", default=None)
    fw.add_argument("--no-firewall", dest="firewall", action="store_false")
    ap.add_argument("--world", default=None, help="Gazebo world name (worlds/<name>.sdf); default simulator.px4_world_name")
    a = ap.parse_args()
    print(generate(yaml.safe_load(open(a.config)), a.out, a.mode, a.firewall, a.world))


if __name__ == "__main__":
    main()
