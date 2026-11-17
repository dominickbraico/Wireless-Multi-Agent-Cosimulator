#!/usr/bin/env python3
"""The `custom` scenario's part of the experiment YAML: which workspace, which launch file, which topics cross the radio.

    scenario:
      type: custom
      custom:
        workspace: examples/template_ws          # colcon workspace (has src/); relative to the repository root, or absolute
        launch: {package: consensus_controller, file: agent.launch.py, args: {gain: 0.3}}
        radio_topics:
          - {name: /swarm/state, type: consensus_msgs/msg/AgentState, reliability: best_effort, durability: volatile, depth: 10}
    agents:
      - {name: agent1, ..., launch_args: {start_altitude: 4.0}}

Pure functions, no side effects: used on the host (cosim.py build, tests) and inside the agent container (scenario_custom.py).
Inside the container the workspace's source is not present, so `load(..., check_files=False)` skips the checks that need it.

    custom_config.py <experiment.yaml>      prints the resolved settings as JSON, or exits 2 with the offending key
See docs/CUSTOM_CONTROLLERS.md.
"""
import json
import os
import re
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
QOS = {"reliability": ("best_effort", "reliable"), "durability": ("volatile", "transient_local")}
QOS_DEFAULT = {"reliability": "best_effort", "durability": "volatile", "depth": 10}
TYPE_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]*/msg/[A-Za-z][A-Za-z0-9_]*$")
NAME_RE = re.compile(r"^(/[A-Za-z_][A-Za-z0-9_]*)+$")
# Topic prefixes that must never reach the radio: PX4's uXRCE-DDS topics (instance 0: /fmu/..., instance N: /px4_N/fmu/...).
PX4_PREFIXES = ("/fmu", "/px4_")
K = "scenario.custom"


def load(cfg, root=ROOT, check_files=True):
    """-> {"workspace": abs path, "launch": {"package", "file", "args"}, "radio_topics": [{name, type, reliability, durability, depth}]}
    Raises ValueError naming the YAML key."""
    c = ((cfg.get("scenario") or {}).get("custom"))
    if not isinstance(c, dict):
        raise ValueError(f"{K}: section missing (scenario.type is custom)")
    unknown = set(c) - {"workspace", "launch", "radio_topics"}
    if unknown:
        raise ValueError(f"{K}: unknown keys {sorted(unknown)}")

    ws = c.get("workspace")
    if not isinstance(ws, str) or not ws:
        raise ValueError(f"{K}.workspace: required, the path of a colcon workspace")
    ws = ws if os.path.isabs(ws) else os.path.join(root, ws)
    launch = c.get("launch")
    if not isinstance(launch, dict):
        raise ValueError(f"{K}.launch: required, {{package, file, args}}")
    unknown = set(launch) - {"package", "file", "args"}
    if unknown:
        raise ValueError(f"{K}.launch: unknown keys {sorted(unknown)}")
    pkg, lfile, largs = launch.get("package"), launch.get("file"), launch.get("args") or {}
    if not isinstance(pkg, str) or not pkg:
        raise ValueError(f"{K}.launch.package: required")
    if not isinstance(lfile, str) or not lfile:
        raise ValueError(f"{K}.launch.file: required")
    if not isinstance(largs, dict):
        raise ValueError(f"{K}.launch.args: must be a mapping of launch arguments")
    if check_files:
        if not os.path.isdir(os.path.join(ws, "src")):
            raise ValueError(f"{K}.workspace: '{ws}' has no src/ directory")
        pkg_dir = _package_dir(os.path.join(ws, "src"), pkg)
        if pkg_dir is None:
            raise ValueError(f"{K}.launch.package: no package '{pkg}' (a directory with a package.xml naming it) under {ws}/src")
        if not any(lfile in files for _, _, files in os.walk(pkg_dir)):
            raise ValueError(f"{K}.launch.file: no file '{lfile}' in package '{pkg}'")

    topics = c.get("radio_topics")
    if not isinstance(topics, list) or not topics:
        raise ValueError(f"{K}.radio_topics: required, a non-empty list (a controller with no radio topic exchanges nothing)")
    out, seen = [], set()
    for i, t in enumerate(topics):
        where = f"{K}.radio_topics[{i}]"
        if not isinstance(t, dict):
            raise ValueError(f"{where}: must be a mapping {{name, type, reliability, durability, depth}}")
        unknown = set(t) - {"name", "type", *QOS_DEFAULT}
        if unknown:
            raise ValueError(f"{where}: unknown keys {sorted(unknown)}")
        name, typ = t.get("name"), t.get("type")
        if not isinstance(name, str) or not NAME_RE.match(name):
            raise ValueError(f"{where}.name: must be an absolute topic name such as /swarm/state, got {name!r}")
        if name.startswith(PX4_PREFIXES):
            raise ValueError(f"{where}.name: {name} is a PX4 topic; PX4 topics stay on the local domain")
        if name in seen:
            raise ValueError(f"{where}.name: duplicate topic {name}")
        seen.add(name)
        if not isinstance(typ, str) or not TYPE_RE.match(typ):
            raise ValueError(f"{where}.type: must look like package/msg/Name, got {typ!r}")
        q = dict(QOS_DEFAULT, **{k: t[k] for k in QOS_DEFAULT if k in t})
        for k, ok in QOS.items():
            if q[k] not in ok:
                raise ValueError(f"{where}.{k}: must be one of {ok}, got {q[k]!r}")
        if not isinstance(q["depth"], int) or isinstance(q["depth"], bool) or q["depth"] < 1:
            raise ValueError(f"{where}.depth: must be an integer >= 1, got {q['depth']!r}")
        out.append({"name": name, "type": typ, **q})
    for a in cfg.get("agents", []):
        if not isinstance(a.get("launch_args") or {}, dict):
            raise ValueError(f"agents[{a.get('name')}].launch_args: must be a mapping of launch arguments")
    return {"workspace": ws, "launch": {"package": pkg, "file": lfile, "args": dict(largs)}, "radio_topics": out}


def _package_dir(src, pkg):
    for d, _, files in os.walk(src):
        if "package.xml" in files and re.search(rf"<name>\s*{re.escape(pkg)}\s*</name>", open(os.path.join(d, "package.xml")).read()):
            return d
    return None


def launch_args(cfg, agent, settings, logs="/logs"):
    """Every launch argument an agent's launch file receives, as strings. Framework arguments first, then `launch.args`,
    then the agent's `launch_args` (later wins)."""
    inst = int(agent["px4_instance"])
    x, y, z, yaw = (float(v) for v in agent["spawn_pose"])
    args = {
        "agent_name": agent["name"],
        "agent_id": inst + 1,                                   # also PX4's MAVLink system id
        "n_agents": len(cfg["agents"]),
        "px4_instance": inst,
        "px4_ns": "" if inst == 0 else f"/px4_{inst}",          # prefix of this vehicle's /fmu/... topics
        "spawn_x": x, "spawn_y": y, "spawn_z": z, "spawn_yaw": yaw,   # world ENU, metres; yaw as in the YAML
        "log_dir": f"{logs}/custom/{agent['name']}",
    }
    args.update(settings["launch"]["args"])
    args.update(agent.get("launch_args") or {})
    return {k: str(v) for k, v in args.items()}


def topics_file_text(settings):
    """The relay's topics file (ros2_ws/src/radio_relay): one `<name> <type> <reliability> <durability> <depth>` per line."""
    return "".join(f"{t['name']} {t['type']} {t['reliability']} {t['durability']} {t['depth']}\n" for t in settings["radio_topics"])


def image_tag(cfg):
    """Docker tag of the overlay agent image for this experiment (docker/agent-custom.Dockerfile)."""
    return "cosim-agent-custom-" + re.sub(r"[^a-z0-9_.-]", "-", str(cfg["experiment"]["name"]).lower())


if __name__ == "__main__":
    try:
        print(json.dumps(load(yaml.safe_load(open(sys.argv[1]))), indent=1))
    except (ValueError, KeyError) as e:
        print(f"custom_config.py: {e}", file=sys.stderr)
        sys.exit(2)
