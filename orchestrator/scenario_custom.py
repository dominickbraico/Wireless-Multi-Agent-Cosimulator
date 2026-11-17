#!/usr/bin/env python3
"""The `custom` scenario, as run by one agent container after PX4 is ready (called from agent_main.py).

Selected by `scenario.type: custom`. The agent runs the user's own ROS 2 launch file (built into the image from
`scenario.custom.workspace`, docker/agent-custom.Dockerfile) on the local DDS domain, next to PX4. The only process on the
radio DDS domain is the framework's relay (ros2_ws/src/radio_relay), which copies the topics listed in
`scenario.custom.radio_topics` between the two domains. See docs/CUSTOM_CONTROLLERS.md.

  1. relay, before the go signal, so that discovery over the radio is done when the user's nodes start
  2. waits for /logs/go
  3. `ros2 launch <package> <file> <arguments>` with ROS_DOMAIN_ID=0 and localhost-only discovery
  4. the run ends for this agent when the launch exits, or `experiment.duration_s` after the go signal (SIGINT, then SIGKILL 10 s later)
  5. writes /logs/<agent>.done and idles
The framework does not land the vehicle: that is the controller's job.
Files in /logs: custom/<agent>/ (relay_topics.txt, and whatever the user's nodes write to `log_dir`), relay/relay_<agent>.csv,
relay_<agent>.log, <agent>_launch.log, <agent>.done.
"""
import os
import signal
import subprocess
import time

import custom_config
import supervise
from supervise import spawn, wait_for


def _last_line(path):
    try:
        lines = [l for l in open(path).read().splitlines() if l.strip()]
        return lines[-1] if lines else "(empty log)"
    except OSError:
        return "(no log)"


def run(cfg, agent, name, inst, logs, dds_dir):
    s = custom_config.load(cfg, check_files=False)        # the workspace's source is on the host, not in the container
    own = f"{logs}/custom/{name}"
    os.makedirs(own, exist_ok=True)
    os.makedirs(f"{logs}/relay", exist_ok=True)
    topics_file = f"{own}/relay_topics.txt"
    open(topics_file, "w").write(custom_config.topics_file_text(s))

    # relay: the only process of this agent on the radio domain (42, bound to radio0 by the Cyclone configuration)
    renv = dict(os.environ, CYCLONEDDS_URI=f"file://{dds_dir}/cyclonedds_agent.xml")
    renv.pop("ROS_AUTOMATIC_DISCOVERY_RANGE", None)       # the relay sets LOCALHOST itself, for its local side only
    relay_log = f"{logs}/relay_{name}.log"
    relay = spawn(["ros2", "run", "radio_relay", "radio_relay", "--ros-args", "-p", f"topics_file:={topics_file}", "-p", f"agent_name:={name}",
                   "-p", f"log_csv:={logs}/relay/relay_{name}.csv"], relay_log, env=renv, label="relay")
    time.sleep(3.0)
    if relay.poll() is not None:                          # an unknown message type or a bad topics file: say so before the go signal
        # quote the relay's own reason: `ros2 run` adds its own last line ("Process exited with failure 2")
        reason = [l for l in open(relay_log, errors="ignore").read().splitlines() if l.startswith("radio_relay:")]
        print(f"agent_main[{name}]: relay failed: {reason[-1] if reason else _last_line(relay_log)}", flush=True)
        supervise.stop_all()

    if not wait_for(lambda: os.path.exists(f"{logs}/go") or supervise.died(), 900):
        print(f"agent_main[{name}]: no go signal within 900 s", flush=True)
        supervise.stop_all()

    lenv = dict(os.environ, ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST", ROS_DOMAIN_ID="0")
    launch_log = f"{logs}/{name}_launch.log"
    # `ros2 launch` refuses an argument with an empty value ("malformed launch argument 'px4_ns:='", first template flight,
    # 2026-11-17), so empty values (px4_ns of PX4 instance 0) are left out and the launch file's declared default applies.
    largs = [f"{k}:={v}" for k, v in custom_config.launch_args(cfg, agent, s, logs).items() if v != ""]
    # not through supervise.spawn: the launch is expected to exit, which must not count as a process that died
    t0 = time.time()
    launch = subprocess.Popen(["ros2", "launch", s["launch"]["package"], s["launch"]["file"], *largs], stdout=open(launch_log, "w"),
                              stderr=subprocess.STDOUT, env=lenv, start_new_session=True)
    duration = float(cfg["experiment"]["duration_s"])
    wait_for(lambda: launch.poll() is not None, duration, period=0.5)
    if launch.poll() is None:
        for sig, grace in ((signal.SIGINT, 10.0), (signal.SIGKILL, 2.0)):
            try:
                os.killpg(launch.pid, sig)
            except ProcessLookupError:
                break
            if wait_for(lambda: launch.poll() is not None, grace, period=0.2):
                break
        print(f"agent_main[{name}]: launch ended (stopped after {duration:.0f} s)", flush=True)
    else:
        rc = launch.returncode
        if rc != 0 and time.time() - t0 < 5.0:
            print(f"agent_main[{name}]: launch failed, see {name}_launch.log: {_last_line(launch_log)}", flush=True)
        print(f"agent_main[{name}]: launch ended (exit {rc})", flush=True)
    open(f"{logs}/{name}.done", "w").close()
    while True:
        time.sleep(60)
