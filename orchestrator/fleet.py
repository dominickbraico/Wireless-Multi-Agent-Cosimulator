#!/usr/bin/env python3
"""Who leads, who follows, and where each follower sits relative to the leader, from the experiment YAML.

Roles: an agent may set `role: leader|follower`. With no role set, the first agent is the leader and every
other agent a follower. Exactly one leader is supported (one LeaderState stream).
Offsets: a follower's formation offset (world ENU metres, follower - leader) is its optional
`follower_offset: [x, y, z]`; otherwise it is the spawn_pose difference, i.e. the formation the agents start in.

    fleet.py <experiment.yaml> [--offset x,y,z]    one line per agent, tab separated:
        role  name  px4_instance  spawn_x  spawn_y  off_x  off_y  off_z
    (--offset overrides every follower's offset; the leader's offset columns are 0.)
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


def main():
    cfg = yaml.safe_load(open(sys.argv[1]))
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
