#!/usr/bin/env python3
"""Per-agent radio plumbing for the ns-3 emulation, driven by the experiment YAML.

For every agent: TAP (opened later by ns-3) + Linux bridge + veth pair, with the
far veth end moved into the agent's network namespace as `radio0`.

    agent netns: radio0 -> veth -> br-<agent> -> tap-<agent> -> ns-3 TapBridge

Backends decide how the agent's namespace is found:
  netns   (default) plain `ip netns` namespaces named ns-<agent>   [local dev]
  (Docker deployments do not use this script: see container_net.py, which reuses the helpers below.)

Needs CAP_NET_ADMIN (root, or `unshare -Urnm` for rootless testing).
Idempotent: `up` tears down first.

    wire.py up|down --config <experiment.yaml>   (e.g. config/experiments/two_agent_wifi.yaml)
"""
import argparse
import subprocess
import sys

import yaml


def sh(cmd, check=True, quiet=False):
    r = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if check and r.returncode != 0:
        sys.exit(f"command failed: {cmd}\n{r.stderr.strip()}")
    return r


class NetnsBackend:
    @staticmethod
    def ns(agent):
        return f"ns-{agent['name']}"

    def create(self, agent):
        sh(f"ip netns add {self.ns(agent)}")

    def exec(self, agent, cmd):
        return f"ip netns exec {self.ns(agent)} {cmd}"

    def move_link(self, agent, dev):
        return f"ip link set {dev} netns {self.ns(agent)}"

    def destroy(self, agent):
        sh(f"ip netns del {self.ns(agent)}", check=False)


BACKENDS = {"netns": NetnsBackend}


def source_mac_guard(macs):
    """Let only each agent's radio0 frames reach its TAP.

    TapBridge (UseLocal) sets the ghost device's MAC to the source of the first
    frame it reads from the TAP. Linux emits its own frames from the bridge/TAP
    (e.g. IGMP reports) which would win that race and leave the ghost with the
    wrong MAC; unicast frames to radio0 would then never match.
    """
    rules = "\n".join(
        f'    oifname "{tap}" ether saddr != {mac} drop' for tap, mac in macs
    )
    ruleset = (
        "table bridge cosim {\n  chain post {\n"
        "    type filter hook postrouting priority 0; policy accept;\n"
        f"{rules}\n  }}\n}}\n"
    )
    r = subprocess.run(["nft", "-f", "-"], input=ruleset, text=True, capture_output=True)
    if r.returncode != 0:
        sys.exit(f"nft failed: {r.stderr.strip()}")


def mgmt_up(cfg, be):
    """Unemulated management network: bridge in the root namespace (simulator side) plus one
    isolated port per agent, so agents reach the simulator but NOT each other."""
    m = cfg["network"].get("mgmt")
    if not m:
        return
    prefix = m["subnet"].split("/")[1]
    br = m["bridge"]
    sh(f"ip link add {br} type bridge")
    sh(f"ip link set {br} type bridge mcast_snooping 0")          # gz-transport discovery is multicast
    sh(f"ip addr add {m['bridge_ip']}/{prefix} dev {br}")
    sh(f"ip link set {br} up")
    sh(f"ip route replace 239.255.0.0/16 dev {br}", check=False)   # gz-transport multicast on the mgmt side
    for i, a in enumerate(cfg["agents"]):
        mb, mi = f"mb{i}", f"mi{i}"
        sh(f"ip link add {mb} type veth peer name {mi}")
        sh(be.move_link(a, mi))
        sh(f"ip link set {mb} master {br}")
        sh(f"bridge link set dev {mb} isolated on")               # no port-to-port forwarding
        sh(f"ip link set {mb} up")
        sh(be.exec(a, f"ip link set {mi} name eth0"))
        sh(be.exec(a, f"ip addr add {a['mgmt_ip']}/{prefix} dev eth0"))
        sh(be.exec(a, "ip link set eth0 up"))
        sh(be.exec(a, "ip route add 239.255.0.0/16 dev eth0"), check=False)
    print(f"wire: mgmt {br} {m['bridge_ip']}/{prefix}, agents isolated from each other")


def radio_firewall_ruleset(cfg):
    """nftables ruleset text for the default-deny policy on radio0, or None when the config has no enabled firewall.
    Shared by the namespace backend (here) and the per-agent containers (container_net.py)."""
    fw = cfg["network"].get("radio_firewall") or {}
    if not fw.get("enabled"):
        return None
    lo, hi = fw["allow_udp_ports"]
    return f"""table inet cosim_radio {{
  chain out {{
    type filter hook output priority 0; policy accept;
    oifname "radio0" udp dport {lo}-{hi} accept
    oifname "radio0" udp dport 7400-7650 counter name leak_domain0_out drop
    oifname "radio0" counter name leak_other_out drop
  }}
  chain in {{
    type filter hook input priority 0; policy accept;
    iifname "radio0" udp dport {lo}-{hi} accept
    iifname "radio0" counter name blocked_in drop
  }}
  counter leak_domain0_out {{ }}
  counter leak_other_out {{ }}
  counter blocked_in {{ }}
}}
"""


def radio_firewall_up(cfg, be):
    """Default-deny on radio0 inside every agent namespace: only the swarm DDS domain's UDP ports may enter or
    leave. Independent of any DDS vendor's behaviour (the XRCE agent, Fast DDS, ignored its loopback-only profile
    and advertised PX4 topics on radio0). Counters on the drop rules report what was blocked."""
    rules = radio_firewall_ruleset(cfg)
    if rules is None:
        return
    lo, hi = cfg["network"]["radio_firewall"]["allow_udp_ports"]
    for a in cfg["agents"]:
        r = subprocess.run(be.exec(a, "nft -f -").split(), input=rules, text=True, capture_output=True)
        if r.returncode != 0:
            sys.exit(f"nft (radio firewall) failed in {a['name']}: {r.stderr.strip()}")
    print(f"wire: radio firewall on (only UDP {lo}-{hi} crosses radio0)")


def veths(i):
    # interface names are limited to 15 chars; derive from index
    return f"vb{i}", f"vi{i}"


def down(cfg, be):
    sh("nft delete table bridge cosim", check=False)
    mg = cfg["network"].get("mgmt")
    if mg:
        sh(f"ip link del {mg['bridge']}", check=False)
        for i in range(len(cfg["agents"])):
            sh(f"ip link del mb{i}", check=False)
    for i, a in enumerate(cfg["agents"]):
        vb, _ = veths(i)
        be.destroy(a)
        sh(f"ip link del {a['bridge']}", check=False)
        sh(f"ip link del {vb}", check=False)
        sh(f"ip link del {a['tap']}", check=False)
    print("wire: down")


def up(cfg, be):
    down(cfg, be)
    prefix = cfg["network"]["radio_subnet"].split("/")[1]
    macs = []
    for i, a in enumerate(cfg["agents"]):
        vb, vi = veths(i)
        tap, br = a["tap"], a["bridge"]
        be.create(a)
        sh(f"ip tuntap add dev {tap} mode tap")
        sh(f"ip link add {br} type bridge")
        sh(f"ip link add {vb} type veth peer name {vi}")
        sh(be.move_link(a, vi))
        # No IPv6 on the emulated chain (keeps neighbour discovery off the channel).
        for d in (tap, br, vb):
            sh(f"sysctl -qw net.ipv6.conf.{d}.disable_ipv6=1", check=False)
        sh(be.exec(a, "sysctl -qw net.ipv6.conf.all.disable_ipv6=1 net.ipv6.conf.default.disable_ipv6=1"))
        sh(f"ip link set {tap} master {br}")
        sh(f"ip link set {vb} master {br}")
        sh(f"ip link set {tap} promisc on up")
        sh(f"ip link set {vb} up")
        sh(f"ip link set {br} up")
        sh(be.exec(a, f"ip link set {vi} name radio0"))
        sh(be.exec(a, f"ip addr add {a['radio_ip']}/{prefix} dev radio0"))
        sh(be.exec(a, "ip link set radio0 mtu 1500 up"))
        sh(be.exec(a, "ip link set lo up"))
        mac = sh(be.exec(a, "cat /sys/class/net/radio0/address")).stdout.strip()
        macs.append((tap, mac))
        print(f"wire: {a['name']} radio0={a['radio_ip']}/{prefix} tap={tap} bridge={br}")
    source_mac_guard(macs)
    mgmt_up(cfg, be)
    radio_firewall_up(cfg, be)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument("action", choices=["up", "down"])
    p.add_argument("--config", required=True)
    p.add_argument("--backend", default="netns", choices=BACKENDS)
    args = p.parse_args()
    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    be = BACKENDS[args.backend]()
    {"up": up, "down": down}[args.action](cfg, be)


if __name__ == "__main__":
    main()
