#!/usr/bin/env python3
"""Host-side driver for the Docker deployment: build the images, start and stop a stack.

    cosim.py build [--config YAML]                  build the images the experiment needs (docker compose build)
    cosim.py up   --config YAML --out DIR [--mode flight|net] [--world W] [--no-firewall] [--timeout S]
                                                     generate the compose project in DIR/compose, start it, wait until healthy
    cosim.py fetch-world W                         download the Fuel models a world needs into the cache (also done by `up`)
    cosim.py down --out DIR                         stop and remove the stack and its networks; hand DIR back to the caller
    cosim.py compose --out DIR -- <docker compose args>   run any docker compose command against the generated project

DIR is the run's log directory: it is mounted at /logs in every container, and the generated compose file is
DIR/compose/docker-compose.yml. Needs only the Docker daemon (no sudo, no host network changes).
"""
import argparse
import os
import subprocess
import sys
import tempfile

import yaml

import compose_gen
import fuel
import simulators

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
DEFAULT_CONFIG = os.path.join(ROOT, "config", "experiments", "two_agent_wifi.yaml")


def compose_file(out):
    return os.path.join(os.path.abspath(out), "compose", "docker-compose.yml")


def compose(out, *args, check=True, **kw):
    cmd = ["docker", "compose", "-f", compose_file(out), *args]
    return subprocess.run(cmd, check=check, **kw)


def cmd_build(a):
    cfg = yaml.safe_load(open(a.config))
    with tempfile.TemporaryDirectory() as tmp:
        path = compose_gen.generate(cfg, tmp, mode="flight")
        subprocess.run(["docker", "compose", "-f", path, "build", *a.services], check=True)


def cmd_up(a):
    cfg = yaml.safe_load(open(a.config))
    os.makedirs(a.out, exist_ok=True)
    if a.mode == "flight" and cfg["simulator"]["type"] == "gazebo":
        world = simulators.get(cfg)["world"](cfg, a.world)
        if os.path.isfile(fuel.world_path(world)):
            fuel.ensure(world)                           # worlds that reference Fuel models need them cached: the stack is offline
            with open(os.path.join(a.out, "fuel_models.txt"), "w") as f:
                f.write("\n".join(fuel.describe(world)) + "\n")
    path = compose_gen.generate(cfg, a.out, a.mode, None if a.firewall else False, a.world)
    print(f"cosim: compose project {path}", flush=True)
    r = compose(a.out, "up", "-d", "--wait", "--wait-timeout", str(a.timeout), check=False)
    if r.returncode != 0:
        print("cosim: stack did not become healthy; container status and logs follow", flush=True)
        compose(a.out, "ps", "-a", check=False)
        compose(a.out, "logs", "--tail", "30", check=False)
    return r.returncode


def cmd_down(a):
    compose(a.out, "down", "--remove-orphans", "--timeout", "30", check=False)
    # containers run as root and wrote into DIR: hand it back to the invoking user
    if os.path.isdir(a.out):
        subprocess.run(["docker", "run", "--rm", "-v", f"{os.path.abspath(a.out)}:/logs", "--entrypoint", "chown",
                        "cosim-ns3:latest", "-R", f"{os.getuid()}:{os.getgid()}", "/logs"], check=False)


def cmd_fetch(a):
    return 0 if fuel.fetch(a.world) else 1


def cmd_compose(a):
    return compose(a.out, *a.args, check=False).returncode


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build")
    b.add_argument("--config", default=DEFAULT_CONFIG)
    b.add_argument("services", nargs="*", help="limit the build to these services (simulator, ns3, agent1, ...)")
    u = sub.add_parser("up")
    u.add_argument("--config", default=DEFAULT_CONFIG)
    u.add_argument("--out", required=True)
    u.add_argument("--mode", choices=["flight", "net"], default="flight")
    u.add_argument("--world", default=None)
    u.add_argument("--no-firewall", dest="firewall", action="store_false", default=True)
    u.add_argument("--timeout", type=int, default=300, help="seconds to wait for every container to be healthy")
    d = sub.add_parser("down")
    d.add_argument("--out", required=True)
    f = sub.add_parser("fetch-world")
    f.add_argument("world")
    c = sub.add_parser("compose")
    c.add_argument("--out", required=True)
    c.add_argument("args", nargs=argparse.REMAINDER)
    a = ap.parse_args()
    if a.cmd == "compose" and a.args[:1] == ["--"]:
        a.args = a.args[1:]
    sys.exit({"build": cmd_build, "up": cmd_up, "down": cmd_down, "fetch-world": cmd_fetch, "compose": cmd_compose}[a.cmd](a) or 0)


if __name__ == "__main__":
    main()
