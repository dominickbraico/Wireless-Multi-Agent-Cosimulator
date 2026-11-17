#!/usr/bin/env python3
"""Network setup run once inside a container at start-up (the Docker counterpart of wire.py).

    container_net.py ns3   --config <yaml>                 in the ns3 container
    container_net.py agent --config <yaml> --name <agent>  in an agent container

ns3 side, per agent:  link-<agent> interface -> Linux bridge -> TAP (opened later by the ns-3 TapBridge), plus the
                      source-MAC guard (the same rule wire.py installs).
agent side:           the link-<agent> interface becomes `radio0` with the agent's radio IP and its planned MAC, the
                      radio firewall (default deny, swarm DDS ports only) and the management-network peer firewall.
                      With `network.mesh.routing: batman_adv` the link interface becomes `mesh0` instead, batman-adv runs
                      on it, and the batman-adv interface is named `radio0` and carries the radio IP. Everything that
                      uses `radio0` (DDS configuration, radio firewall, tests) therefore works unchanged; `mesh0` carries
                      only batman-adv frames.

Interfaces are found by the address Docker assigned (containers.plan), never by name: Docker numbers them in no
guaranteed order. Needs CAP_NET_ADMIN. Addressing comes from containers.plan(), shared with the compose generator.
"""
import argparse
import json
import os
import subprocess
import sys

import yaml

import containers
import wire
from wire import sh


def iface_with_ip(ip):
    out = json.loads(sh("ip -j -4 addr show").stdout)
    for link in out:
        if any(a["local"] == ip for a in link.get("addr_info", [])):
            return link["ifname"]
    sys.exit(f"container_net: no interface carries {ip}; Docker did not attach the expected network")


def nft(ruleset, what):
    r = subprocess.run(["nft", "-f", "-"], input=ruleset, text=True, capture_output=True)
    if r.returncode != 0:
        sys.exit(f"nft ({what}) failed: {r.stderr.strip()}")


def ns3_side(cfg):
    plan = containers.plan(cfg)
    macs = []
    for a in cfg["agents"]:
        p = plan["agents"][a["name"]]
        dev, tap, br = iface_with_ip(p["ns3_link_ip"]), a["tap"], a["bridge"]
        sh(f"ip addr flush dev {dev}")
        sh(f"ip link add {br} type bridge")
        sh(f"ip tuntap add dev {tap} mode tap")
        for d in (tap, br, dev):                          # no IPv6 on the emulated chain (see wire.py)
            sh(f"sysctl -qw net.ipv6.conf.{d}.disable_ipv6=1", check=False)
        sh(f"ip link set {tap} master {br}")
        sh(f"ip link set {dev} master {br}")
        sh(f"ip link set {tap} promisc on up")
        sh(f"ip link set {dev} up")
        sh(f"ip link set {br} up")
        macs.append((tap, p["mac"]))
        print(f"container_net[ns3]: {a['name']} {dev} -> {br} -> {tap} (agent MAC {p['mac']})", flush=True)
    wire.source_mac_guard(macs)


def mgmt_isolation_ruleset(plan, own_ip):
    """Agents share the management bridge with the simulator but must not reach each other (the netns deployment gets
    this from isolated bridge ports). Only the simulator and the agent itself may use management-subnet addresses."""
    m = plan["mgmt"]
    return f"""table inet cosim_mgmt {{
  chain out {{
    type filter hook output priority 0; policy accept;
    ip daddr {m['subnet']} ip daddr != {{ {m['sim_ip']}, {own_ip} }} counter name peer_out drop
  }}
  chain in {{
    type filter hook input priority 0; policy accept;
    ip saddr {m['subnet']} ip saddr != {{ {m['sim_ip']}, {own_ip} }} counter name peer_in drop
  }}
  counter peer_out {{ }}
  counter peer_in {{ }}
}}
"""


def agent_side(cfg, name):
    plan = containers.plan(cfg)
    p = plan["agents"][name]
    prefix = cfg["network"]["radio_subnet"].split("/")[1]
    link = iface_with_ip(p["agent_link_ip"])
    mgmt = iface_with_ip(p["mgmt_ip"])
    m = containers.mesh(cfg)
    phy = "mesh0" if m else "radio0"                     # the interface whose frames enter ns-3
    sh(f"ip link set {link} down")
    sh(f"ip link set {link} name {phy}")
    sh(f"ip link set {phy} address {p['mac']}")          # the source-MAC guard in the ns3 container expects this MAC
    sh(f"ip addr flush dev {phy}")
    if m:
        mesh_up(p, prefix, m)
    else:
        sh(f"ip addr add {p['radio_ip']}/{prefix} dev radio0")
        sh("ip link set radio0 mtu 1500 up")
    sh("ip link set lo up")
    sh(f"ip route add 239.255.0.0/16 dev {mgmt}", check=False)       # gz-transport multicast discovery: management side
    rules = wire.radio_firewall_ruleset(cfg)
    if rules:
        nft(rules, "radio firewall")
    if m:                                                # whether or not the radio firewall is on: mesh0 must never carry IP
        nft(MESH_PHY_RULESET, "mesh interface firewall")
    nft(mgmt_isolation_ruleset(plan, p["mgmt_ip"]), "management isolation")
    print(f"container_net[{name}]: radio0 {p['radio_ip']}/{prefix} mac {p['mac']}; mgmt on {mgmt} {p['mgmt_ip']}; "
          f"radio firewall {'on' if rules else 'off'}; mgmt peers blocked"
          + (f"; batman-adv on mesh0, originator interval {m['orig_interval_ms']} ms" if m else ""), flush=True)


# mesh0 carries batman-adv frames only (ethertype 0x4305, not IP, so an inet table never sees them). Any IP packet on it would
# bypass radio0's firewall, so it is dropped and counted.
MESH_PHY_RULESET = """table inet cosim_mesh {
  chain out {
    type filter hook output priority 0; policy accept;
    oifname "mesh0" counter name ip_on_mesh0_out drop
  }
  chain in {
    type filter hook input priority 0; policy accept;
    iifname "mesh0" counter name ip_on_mesh0_in drop
  }
  counter ip_on_mesh0_out { }
  counter ip_on_mesh0_in { }
}
"""


def mesh_up(p, prefix, m):
    """batman-adv (the host's kernel module; containers cannot load it) on mesh0, with its interface named radio0.
    MTU: batman-adv adds up to 32 bytes per frame (unicast_4addr header plus the inner Ethernet header). The emulated chain
    is 1500 bytes end to end (Docker link network, ns-3 Wi-Fi device), so the IP interface gets 1500 - 32 = 1468 and no
    batman-adv fragmentation is needed; larger IP packets are fragmented by IP."""
    if not os.path.isdir("/sys/module/batman_adv"):
        sys.exit("container_net: the batman_adv kernel module is not loaded on the host (docs/CONTAINERS.md, 'batman-adv mesh routing')")
    sh("ip link set mesh0 mtu 1500 up")
    sh("ip link add radio0 type batadv")
    sh("ip link set mesh0 master radio0")
    sh(f"ip link set radio0 address {p['mesh_mac']}")
    sh("sysctl -qw net.ipv6.conf.radio0.disable_ipv6=1", check=False)      # no IPv6 on the emulated chain
    sh(f"ip addr add {p['radio_ip']}/{prefix} dev radio0")
    sh("ip link set radio0 mtu 1468 up")
    sh(f"batctl meshif radio0 orig_interval {m['orig_interval_ms']}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("side", choices=["ns3", "agent"])
    ap.add_argument("--config", required=True)
    ap.add_argument("--name")
    a = ap.parse_args()
    cfg = yaml.safe_load(open(a.config))
    if a.side == "ns3":
        ns3_side(cfg)
    else:
        if not a.name:
            sys.exit("agent side needs --name")
        agent_side(cfg, a.name)


if __name__ == "__main__":
    main()
