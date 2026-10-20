#!/usr/bin/env bash
# Simulator container: the multicast route gz-transport discovery needs on the (gateway-less) management network,
# then the supervised simulator + pose adapter.
set -eu
dev="$(ip -j -4 addr show | python3 -c "import json,sys,os; ip=os.environ['GZ_IP']; print(next(l['ifname'] for l in json.load(sys.stdin) if any(a['local']==ip for a in l['addr_info'])))")"
ip route add 239.255.0.0/16 dev "$dev" 2>/dev/null || true
export PYTHONPATH="/opt/cosim/orchestrator:${PYTHONPATH:-}"
exec python3 /opt/cosim/orchestrator/sim_main.py
