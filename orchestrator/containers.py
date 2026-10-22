#!/usr/bin/env python3
"""Addressing and naming for the Docker deployment, derived from the experiment YAML.

One function, `plan(cfg)`, is the single place that decides which IP, subnet and MAC every container gets. The compose
generator (compose_gen.py) and the in-container setup (container_net.py) both call it, so they cannot disagree.

Network layout (see docs/CONTAINERS.md):

    mgmt            one bridge network: simulator, ns3 and every agent. Not emulated. Agents are firewalled from each other.
    link-<agent>    one point-to-point network per agent: only that agent and ns3. Carries the emulated radio frames.

Optional YAML section (all keys optional):

    containers:
      ns3_mgmt_ip: 10.20.0.2          # ns3 container on the management network (default: first free address after bridge_ip)
      link_subnet_base: 10.30.0.0/16  # agent i gets the i-th /24 of this block as its private link to ns3

`containers.py <experiment.yaml>` prints the plan as JSON.
"""
import ipaddress
import json
import sys

import yaml

DEFAULT_LINK_BASE = "10.30.0.0/16"


def plan(cfg):
    net = cfg["network"]
    mgmt = net["mgmt"]
    c = cfg.get("containers") or {}
    subnet = ipaddress.ip_network(mgmt["subnet"])
    sim_ip = mgmt["bridge_ip"]
    used = {sim_ip} | {a["mgmt_ip"] for a in cfg["agents"]}
    gateway = str(subnet.broadcast_address - 1)       # the host side of the Docker bridge; kept off the .1 the simulator uses
    used.add(gateway)
    ns3_ip = c.get("ns3_mgmt_ip")
    if ns3_ip is None:
        ns3_ip = next(str(h) for h in subnet.hosts() if str(h) not in used)
    elif ns3_ip in used:
        raise ValueError(f"containers.ns3_mgmt_ip {ns3_ip} collides with another management address")

    base = ipaddress.ip_network(c.get("link_subnet_base", DEFAULT_LINK_BASE))
    links = list(base.subnets(new_prefix=24))
    if len(cfg["agents"]) > len(links) - 1:
        raise ValueError(f"link_subnet_base {base} has room for {len(links) - 1} agents, got {len(cfg['agents'])}")
    agents = {}
    for i, a in enumerate(cfg["agents"]):
        ln = links[i + 1]                              # skip the first /24: Docker may keep it for its own use
        agents[a["name"]] = {
            "index": i,
            "link_net": f"link-{a['name']}",
            "link_subnet": str(ln),
            "link_gateway": str(ln.network_address + 254),
            "agent_link_ip": str(ln.network_address + 10),
            "ns3_link_ip": str(ln.network_address + 11),
            # locally administered unicast MAC; ns-3's source-MAC guard (wire.source_mac_guard) is built from it
            "mac": "02:c0:5d:00:%02x:%02x" % ((i + 1) >> 8, (i + 1) & 0xFF),
            "mgmt_ip": a["mgmt_ip"],
            "radio_ip": a["radio_ip"],
        }
    return {
        "mgmt": {"network": "mgmt", "subnet": str(subnet), "gateway": gateway, "sim_ip": sim_ip, "ns3_ip": ns3_ip},
        "agents": agents,
    }


if __name__ == "__main__":
    print(json.dumps(plan(yaml.safe_load(open(sys.argv[1]))), indent=2))
