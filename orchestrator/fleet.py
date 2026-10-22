#!/usr/bin/env python3
"""Who leads, who follows, and where each follower sits relative to the leader, from the experiment YAML.

Roles: an agent may set `role: leader|follower`. With no role set, the first agent is the leader and every
other agent a follower. Exactly one leader is supported (one LeaderState stream).
Offsets: a follower's formation offset (world ENU metres, follower - leader) is its optional
`follower_offset: [x, y, z]`; otherwise it is the spawn_pose difference, i.e. the formation the agents start in.

    fleet.py <experiment.yaml> [--offset x,y,z]    one line per agent, tab separated:
        role  name  px4_instance  spawn_x  spawn_y  off_x  off_y  off_z
    (--offset overrides every follower's offset; the leader's offset columns are 0.)

    fleet.py <experiment.yaml> --qos-args      ros2 `-p` arguments that set the LeaderState QoS from `swarm.qos.leader_state`
"""
import sys

import yaml


def resolve(cfg, offset_override=None):
    """-> (leader_agent, [(follower_agent, [ox, oy, oz]), ...])"""
    agents = cfg["agents"]
    leaders = [a for a in agents if a.get("role") == "leader"] or [agents[0]]
    if len(leaders) != 1:
        raise ValueError(f"exactly one leader is supported, got {[a['name'] for a in leaders]}")
    leader = leaders[0]
    followers = []
    for a in agents:
        if a is leader:
            continue
        if a.get("role") not in (None, "follower"):
            raise ValueError(f"{a['name']}: role must be leader or follower, got {a['role']}")
        if offset_override is not None:
            off = list(offset_override)
        elif a.get("follower_offset") is not None:
            off = [float(v) for v in a["follower_offset"]]
        else:
            off = [float(a["spawn_pose"][0]) - float(leader["spawn_pose"][0]),
                   float(a["spawn_pose"][1]) - float(leader["spawn_pose"][1]), 0.0]
        followers.append((a, off))
    return leader, followers


QOS_KEYS = {"reliability": ("best_effort", "reliable"), "durability": ("volatile", "transient_local")}
# peer_state is a keyless topic with several publishers: a subscriber's keep-last history is shared by all senders, so with depth 1
# heartbeats that arrive together overwrite each other and the earliest sender loses most (seen in the first mesh-dropout run).
DEFAULT_DEPTH = {"leader_state": 1, "peer_state": 10}


def topic_qos(cfg, topic):
    """-> {reliability, durability, depth} for a radio topic. YAML `swarm.qos.<topic>`; defaults best_effort, volatile, and depth 1 (10 for peer_state: see DEFAULT_DEPTH).
    Topics: `leader_state` (leader-follower scenario), `peer_state` (mesh-dropout scenario)."""
    q = {"reliability": "best_effort", "durability": "volatile", "depth": DEFAULT_DEPTH.get(topic, 1)}
    given = (((cfg.get("swarm") or {}).get("qos") or {}).get(topic)) or {}
    unknown = set(given) - set(q)
    if unknown:
        raise ValueError(f"swarm.qos.{topic}: unknown keys {sorted(unknown)}")
    q.update(given)
    for k, ok in QOS_KEYS.items():
        if q[k] not in ok:
            raise ValueError(f"swarm.qos.{topic}.{k} must be one of {ok}, got {q[k]!r}")
    if not isinstance(q["depth"], int) or q["depth"] < 1:
        raise ValueError(f"swarm.qos.{topic}.depth must be an integer >= 1, got {q['depth']!r}")
    return q


def leader_state_qos(cfg):
    return topic_qos(cfg, "leader_state")


def qos_args(cfg, topic="leader_state"):
    """The same, as ros2 command-line arguments (used by the container supervisor and the shell tests)."""
    q = topic_qos(cfg, topic)
    return ["-p", f"qos_reliability:={q['reliability']}", "-p", f"qos_durability:={q['durability']}",
            "-p", f"qos_depth:={q['depth']}"]


def main():
    cfg = yaml.safe_load(open(sys.argv[1]))
    if "--qos-args" in sys.argv:
        print(" ".join(qos_args(cfg)))
        return
    override = None
    if "--offset" in sys.argv:
        override = [float(v) for v in sys.argv[sys.argv.index("--offset") + 1].split(",")]
    leader, followers = resolve(cfg, override)
    row = lambda role, a, off: print("\t".join(str(v) for v in (
        role, a["name"], a["px4_instance"], float(a["spawn_pose"][0]), float(a["spawn_pose"][1]), *off)))   # floats: ROS rejects [0, 8] for a double-array parameter
    row("leader", leader, [0.0, 0.0, 0.0])
    for a, off in followers:
        row("follower", a, off)


if __name__ == "__main__":
    try:
        main()
    except (ValueError, KeyError) as e:
        sys.exit(f"fleet.py: {e}")
