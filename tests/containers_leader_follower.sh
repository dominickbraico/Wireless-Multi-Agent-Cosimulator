#!/usr/bin/env bash
# Leader-follower flight in Docker Compose: a simulator container, an ns-3 container and one container per agent
# (PX4 SITL + ROS 2). The container counterpart of tests/leader_follower_fly.sh; same flight, same analysis, same pass criteria.
#   tests/containers_leader_follower.sh [--config YAML]      (images first: orchestrator/cosim.py build)
# Env: PX4_WORLD (worlds/<name>.sdf, default from the YAML), OUT (log directory), SIDE (m), ALT (m), SPEED (m/s),
#      OFFSET (x,y,z: force this offset on every follower)
# Needs only the Docker daemon (no sudo, no host network changes). The Gazebo GUI is not part of the stack.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
. "$ROOT/tests/_config_flag.sh"          # sets CFG from --config <yaml> (default: two_agent_wifi.yaml)
OUT="${OUT:-$(mktemp -d)}"; OFFSET="${OFFSET:-}"
export SIDE="${SIDE:-10.0}" ALT="${ALT:-5.0}" SPEED="${SPEED:-1.5}" OFFSET
cosim() { python3 "$ROOT/orchestrator/cosim.py" "$@"; }
dc() { docker compose -f "$OUT/compose/docker-compose.yml" "$@"; }   # $ROOT may contain spaces
trap 'cosim down --out "$OUT" >/dev/null 2>&1' EXIT
trap 'exit 130' INT TERM HUP
AGENTS=$(python3 -c "import yaml,sys; print(' '.join(a['name'] for a in yaml.safe_load(open(sys.argv[1]))['agents']))" "$CFG")
N=$(echo $AGENTS | wc -w)
echo "logs=$OUT agents=$N square=${SIDE}m alt=${ALT}m speed=${SPEED}m/s world=${PX4_WORLD:-from YAML}"

cosim up --config "$CFG" --out "$OUT" --mode flight ${PX4_WORLD:+--world "$PX4_WORLD"} --timeout "$((150 + 30 * N))" > "$OUT/up.log" 2>&1 \
  || { tail -40 "$OUT/up.log"; echo "FAIL  stack not healthy (see $OUT/up.log)"; exit 1; }
echo "stack up: simulator, ns3 and $N agent containers healthy"
sleep 8                                   # EKF settle + first poses reach ns-3
touch "$OUT/go"                           # every agent supervisor waits for this file

for i in $(seq 1 600); do
  left=0; for a in $AGENTS; do [ -e "$OUT/$a.done" ] || left=1; done
  [ $left = 0 ] && break; sleep 1
done
[ $left = 0 ] || echo "WARN  not every agent finished within 600 s"
sleep 2
# what the radio firewall had to block (attempted leaks), and what the management firewall blocked, before the containers go
for a in $AGENTS; do echo "firewall $a: $(dc exec -T "$a" nft list counters table inet cosim_radio 2>/dev/null | tr -s ' \n' ' ' | sed 's/counter //g')"; done | tee "$OUT/firewall_counters.txt"
for a in $AGENTS; do echo "mgmt-isolation $a: $(dc exec -T "$a" nft list counters table inet cosim_mgmt 2>/dev/null | tr -s ' \n' ' ' | sed 's/counter //g')"; done | tee "$OUT/mgmt_firewall_counters.txt"
dc stop ns3 >/dev/null 2>&1               # SIGTERM: ns-3 writes its summary
dc stop simulator >/dev/null 2>&1
dc logs --no-color > "$OUT/containers.log" 2>&1
python3 "$ROOT/tools/analyze_formation.py" "$OUT" "$OUT/compose/experiment_run.yaml" ${OFFSET:+--offset "$OFFSET"}; rc=$?
echo "logs: $OUT"; exit $rc
