#!/usr/bin/env python3
"""Minimal process supervision for the container entry programs (agent_main.py, sim_main.py)."""
import os
import signal
import subprocess
import sys
import time

procs = []     # (Popen, label)


def spawn(cmd, log, env=None, cwd=None, label=None):
    """Start cmd in its own process group with stdout/stderr appended to log."""
    f = open(log, "a")
    p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, env=env, cwd=cwd, start_new_session=True)
    procs.append((p, label or os.path.basename(cmd[0])))
    return p


def kill_group(p, sig=signal.SIGTERM):
    """Signal the whole process group: `ros2 run` is only a launcher and leaves the node running if it alone is signalled."""
    try:
        os.killpg(p.pid, sig)
    except ProcessLookupError:
        pass


def stop_all(*_):
    for p, _ in reversed(procs):
        kill_group(p)
    time.sleep(1.0)
    for p, _ in procs:
        kill_group(p, signal.SIGKILL)
    sys.exit(0)


def install_handlers():
    signal.signal(signal.SIGTERM, stop_all)
    signal.signal(signal.SIGINT, stop_all)


def died():
    """Labels of supervised processes that have exited."""
    return [(label, p.returncode) for p, label in procs if p.poll() is not None]


def wait_for(predicate, timeout, period=0.5):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if predicate():
            return True
        time.sleep(period)
    return False


def file_contains(path, text):
    try:
        return text in open(path, errors="ignore").read()
    except FileNotFoundError:
        return False
