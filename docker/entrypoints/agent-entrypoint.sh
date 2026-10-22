#!/usr/bin/env bash
# Agent container: wire up the radio interface and firewalls (needs NET_ADMIN), then run the per-agent stack.
set -e
CFG="${COSIM_CONFIG:-/config/experiment_run.yaml}"
python3 /opt/cosim/orchestrator/container_net.py agent --config "$CFG" --name "$COSIM_AGENT"
set +u
source /opt/ros/jazzy/setup.bash
source /opt/cosim_ws/install/setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp PYTHONPATH="/opt/cosim/orchestrator:${PYTHONPATH:-}"
exec python3 /opt/cosim/orchestrator/agent_main.py
