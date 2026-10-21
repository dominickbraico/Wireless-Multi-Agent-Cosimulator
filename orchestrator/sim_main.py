#!/usr/bin/env python3
"""Entry program of the simulator container: run the physics simulator and its pose adapter, supervised.

    sim_main.py            reads COSIM_CONFIG (the experiment YAML) and the simulator.type driver in simulators.py
Both processes log to /logs; the container exits if either dies.
"""
import os
import sys
import time

import yaml

import simulators
import supervise

LOGS = "/logs"


def main():
    config = os.environ.get("COSIM_CONFIG", "/config/experiment_run.yaml")
    cfg = yaml.safe_load(open(config))
    sim = simulators.get(cfg)
    world = os.environ.get("COSIM_WORLD") or sim["world"](cfg)
    supervise.install_handlers()
    env = dict(os.environ, PATH="/usr/bin:" + os.environ["PATH"], GZ_CONFIG_PATH="/usr/share/gz",
               GZ_SIM_RESOURCE_PATH="/opt/simulation-gazebo/models:/opt/simulation-gazebo/worlds:/opt/cosim/worlds/models",
               GZ_SIM_SERVER_CONFIG_PATH="/opt/simulation-gazebo/server.config")
    os.makedirs(f"{LOGS}/bridge", exist_ok=True)
    for name, argv in sim["processes"](cfg, world, config, LOGS):
        supervise.spawn(argv, f"{LOGS}/{name}.log", env=env, label=name)
        time.sleep(2)
    print(f"sim_main: {cfg['simulator']['type']} world {world} running", flush=True)
    while True:
        gone = supervise.died()
        if gone:
            print(f"sim_main: process exited: {gone}", flush=True)
            supervise.stop_all()
            sys.exit(1)
        time.sleep(2)


if __name__ == "__main__":
    main()
