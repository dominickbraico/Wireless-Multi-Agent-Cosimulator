#!/usr/bin/env bash
# ns3 container: build the per-agent bridges and TAPs (needs NET_ADMIN), then exec the scenario for the chosen network
# technology so that SIGTERM from `docker stop` reaches ns-3 and it writes its summary.
set -eu
CFG="${COSIM_CONFIG:-/config/experiment_run.yaml}"
rm -f /logs/.ns3_ready
python3 /opt/cosim/orchestrator/container_net.py ns3 --config "$CFG"
mkdir -p /logs/ns3
# "ready" once the real-time loop has ticked (lag.csv gets a row per simulated second)
( for i in $(seq 1 120); do [ -f /logs/ns3/lag.csv ] && [ "$(wc -l < /logs/ns3/lag.csv)" -ge 2 ] && { touch /logs/.ns3_ready; exit 0; }; sleep 1; done ) &
exec "/opt/cosim/scenarios/${COSIM_SCENARIO}" --config="$CFG" --log-dir=/logs/ns3 --duration="${NS3_DURATION:-3600}"
