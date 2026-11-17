#!/usr/bin/env bash
# Isaac Sim container: the supervised simulator process (orchestrator/sim_main.py), run with Isaac Sim's own Python.
set -eu
export PYTHONPATH="/opt/cosim/orchestrator:/opt/pegasus/extensions/pegasus.simulator:/opt/pylib:${PYTHONPATH:-}"
exec /isaac-sim/python.sh /opt/cosim/orchestrator/sim_main.py
