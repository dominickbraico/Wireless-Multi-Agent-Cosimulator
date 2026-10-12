#!/usr/bin/env python3
"""Start / supervise the simulator + per-agent PX4 stack described by the experiment YAML.

    sudo orchestrator/run_stack.py --config <yaml> --log-dir <dir>        # (or inside tests/run_rootless.sh)

Needs `orchestrator/wire.py up` to have run (namespaces + networks). Starts, in order:
  1. Gazebo server (headless) in the root namespace, bound to the management bridge IP
  2. per agent, inside its namespace: Micro XRCE-DDS Agent (Fast DDS, loopback only) and PX4 SITL
     (standalone mode, instance N, spawn pose from the YAML)
Waits until every PX4 reports "Ready for takeoff", then blocks until SIGTERM/SIGINT and stops everything.
Runtime pieces come from the PX4 runtime directory (default ~/dev/px4-runtime, see tools/setup_px4_runtime.sh).
"""
import argparse
import os
import signal
import subprocess
import sys
import time

import yaml

HOME = os.path.expanduser("~" + os.environ["SUDO_USER"]) if os.environ.get("SUDO_USER") else os.path.expanduser("~")
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
procs = []


PIDS_FILE = None


def spawn(cmd, log, env=None, cwd=None):
    f = open(log, "w")
    p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, env=env, cwd=cwd, start_new_session=True)
    procs.append((p, cmd[0]))
    if PIDS_FILE:                       # lets the caller reap orphans if this supervisor dies without cleaning up
        with open(PIDS_FILE, "a") as pf:
            pf.write(f"{p.pid}\n")
    return p


def stop_all(*_):
    for p, _ in reversed(procs):
        try:
            os.killpg(p.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    time.sleep(1.0)
    for p, _ in procs:
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    sys.exit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--log-dir", required=True)
    ap.add_argument("--runtime", default=os.environ.get("PX4_RUNTIME", f"{HOME}/dev/px4-runtime"))
    ap.add_argument("--ready-timeout", type=float, default=90)
    a = ap.parse_args()
    cfg = yaml.safe_load(open(a.config))
    os.makedirs(a.log_dir, exist_ok=True)
    global PIDS_FILE
    PIDS_FILE = os.path.join(a.log_dir, "stack.pids")
    open(PIDS_FILE, "w").close()
    sim, mg = cfg["simulator"], cfg["network"]["mgmt"]
    world, part = sim["world_name"], sim["gz_partition"]
    px4_world = sim.get("px4_world_name", "px4_flat")
    signal.signal(signal.SIGTERM, stop_all)
    signal.signal(signal.SIGINT, stop_all)

    # ROS 2's gz_tools_vendor stub shadows the real `gz` when ROS is sourced. PX4's own
    # startup script runs `gz --versions`, so EVERY child needs the real one first on PATH.
    base = dict(os.environ, PATH="/usr/bin:" + os.environ["PATH"], GZ_CONFIG_PATH="/usr/share/gz")

    # 1. Gazebo server, simulator side of the management network
    genv = dict(base, GZ_PARTITION=part, GZ_IP=mg["bridge_ip"],
                GZ_SIM_RESOURCE_PATH=f"{HOME}/.simulation-gazebo/models:{HOME}/.simulation-gazebo/worlds",
                GZ_SIM_SERVER_CONFIG_PATH=f"{HOME}/.simulation-gazebo/server.config")
    spawn(["/usr/bin/gz", "sim", "-s", "-r", "--headless-rendering", f"{ROOT}/worlds/{px4_world}.sdf"],
          f"{a.log_dir}/gz.log", env=genv)
    time.sleep(8)

    # 2. per-agent XRCE agent + PX4
    b = f"{a.runtime}/px4_sitl_default"
    for ag in cfg["agents"]:
        n, name = ag["px4_instance"], ag["name"]
        ns = ["ip", "netns", "exec", f"ns-{name}"]
        x, y, z, yaw = ag["spawn_pose"]
        aenv = dict(base, LD_LIBRARY_PATH=f"{a.runtime}/lib",
                    FASTRTPS_DEFAULT_PROFILES_FILE=f"{ROOT}/config/dds/fastdds_loopback.xml")
        subprocess.run(ns + ["ip", "link", "set", "lo", "up"], check=True)
        spawn(ns + [f"{a.runtime}/MicroXRCEAgent", "udp4", "-p", "8888"], f"{a.log_dir}/{name}_xrce.log", env=aenv)
        penv = dict(base, LD_LIBRARY_PATH=f"{a.runtime}/lib", PX4_GZ_STANDALONE="1",
                    PX4_SYS_AUTOSTART="4001", PX4_SIM_MODEL="gz_x500", PX4_GZ_WORLD=px4_world,
                    GZ_PARTITION=part, GZ_IP=ag["mgmt_ip"], PX4_GZ_MODEL_POSE=f"{x},{y},{z},0,0,{yaw}")
        wd = f"{b}/instance_{n}"
        os.makedirs(wd, exist_ok=True)
        # PX4 keeps its daemon socket and lock in /tmp (px4-sock-N, px4_lock-N). Each PX4 gets a PRIVATE /tmp so a stale
        # file from an earlier (possibly root-owned or killed) run can never block it, as separate containers would have.
        spawn(ns + ["unshare", "-m", "sh", "-c", 'mount -t tmpfs tmpfs /tmp && exec "$@"', "px4",
                    f"{b}/bin/px4", "-i", str(n), "-d", f"{b}/etc"], f"{a.log_dir}/{name}_px4.log", env=penv, cwd=wd)

    # 3. wait for readiness
    t0 = time.time()
    pending = {ag["name"] for ag in cfg["agents"]}
    while pending and time.time() - t0 < a.ready_timeout:
        for name in list(pending):
            try:
                if "Ready for takeoff" in open(f"{a.log_dir}/{name}_px4.log", errors="ignore").read():
                    pending.discard(name)
                    print(f"run_stack: {name} PX4 ready ({time.time() - t0:.0f}s)", flush=True)
            except FileNotFoundError:
                pass
        time.sleep(1)
    if pending:
        print(f"run_stack: NOT READY within {a.ready_timeout}s: {sorted(pending)}", flush=True)
        stop_all()
    print("run_stack: ALL READY", flush=True)
    while True:
        for p, c in procs:
            if p.poll() is not None:
                print(f"run_stack: process exited early: {c} rc={p.returncode}", flush=True)
        time.sleep(2)


if __name__ == "__main__":
    main()
