#!/usr/bin/env bash
# batman-adv transition flight in Docker Compose: agent3 flies out of radio range of agent1 while staying in range of agent2, holds,
# and returns; batman-adv in every agent reroutes agent1 <-> agent3 through agent2 and back. agent1 pings agent3 at 5 Hz throughout.
# Criteria T1-T5: tools/analyze_batman.py and docs/EXPERIMENTS.md, "batman-adv mesh".
#   tests/containers_batman_transition.sh [--config YAML]      (default batman_transition.yaml)
# Needs: the batman_adv kernel module on the host (docs/CONTAINERS.md), images (python3 orchestrator/cosim.py build).
# Env: WAIT_FOR_GUI=1 (hold the flight until $OUT/go exists; used by the demo), PX4_WORLD, OUT (log directory)
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CFG="$ROOT/config/experiments/batman_transition.yaml"
. "$ROOT/tests/_config_flag.sh"
OUT="${OUT:-$ROOT/logs/batman_transition_$(date +%Y%m%d_%H%M%S)}"; mkdir -p "$OUT/capture"
cosim() { python3 "$ROOT/orchestrator/cosim.py" "$@"; }
dc() { docker compose -f "$OUT/compose/docker-compose.yml" "$@"; }
trap 'cosim down --out "$OUT" >/dev/null 2>&1' EXIT
trap 'exit 130' INT TERM HUP
python3 "$ROOT/orchestrator/mesh_plan.py" "$CFG" > "$OUT/plan_preview.json" || { echo "FAIL  the YAML does not give a valid plan"; exit 2; }
AGENTS=$(python3 -c "import yaml,sys; print(' '.join(a['name'] for a in yaml.safe_load(open(sys.argv[1]))['agents']))" "$CFG")
N=$(echo $AGENTS | wc -w)
DST_IP=$(python3 -c "import yaml,sys; print(yaml.safe_load(open(sys.argv[1]))['agents'][2]['radio_ip'])" "$CFG")
echo "logs=$OUT agents=$N"

cosim up --config "$CFG" --out "$OUT" --mode flight ${PX4_WORLD:+--world "$PX4_WORLD"} --timeout "$((150 + 30 * N))" > "$OUT/up.log" 2>&1 \
  || { tail -40 "$OUT/up.log"; echo "FAIL  stack not healthy (see $OUT/up.log)"; exit 1; }
echo "stack up: simulator, ns3 and $N agent containers healthy"
for a in $AGENTS; do      # captures inside each agent: radio0 = IP before batman-adv encapsulation, mesh0 = what enters ns-3
  dc exec -d "$a" tcpdump -i radio0 -U -w "/logs/capture/${a}_radio0.pcap"
  dc exec -d "$a" tcpdump -i mesh0 -U -w "/logs/capture/${a}_mesh0.pcap"
done
sleep 8                                   # EKF settle + first poses reach ns-3
if [ "${WAIT_FOR_GUI:-0}" = 1 ]; then
  echo "=== All vehicles are up and waiting on the ground. Start the flight with:   touch \"$OUT/go\"      (auto-starts after 300 s)"
  for i in $(seq 1 300); do [ -e "$OUT/go" ] && break; sleep 1; done
fi
dc exec -d agent1 sh -c "ping -D -i 0.2 -W 1 $DST_IP > /logs/ping_agent1_agent3.txt 2>&1"
touch "$OUT/go"

for i in $(seq 1 600); do
  left=0; for a in $AGENTS; do [ -e "$OUT/$a.done" ] || left=1; done
  [ $left = 0 ] && break; sleep 1
done
[ $left = 0 ] || echo "WARN  not every agent finished within 600 s"
dc exec -T agent1 pkill -INT -x ping                  # ping prints its totals on SIGINT
sleep 2
for a in $AGENTS; do dc exec -T "$a" pkill -INT -x tcpdump; done
for a in $AGENTS; do echo "firewall $a: $(dc exec -T "$a" nft list counters table inet cosim_radio 2>/dev/null | tr -s ' \n' ' ' | sed 's/counter //g')"; done | tee "$OUT/firewall_counters.txt"
for a in $AGENTS; do echo "mesh0 $a: $(dc exec -T "$a" nft list counters table inet cosim_mesh 2>/dev/null | tr -s ' \n' ' ' | sed 's/counter //g')"; done | tee "$OUT/mesh_firewall_counters.txt"
dc stop ns3 >/dev/null 2>&1               # SIGTERM: ns-3 writes its summary
dc stop simulator >/dev/null 2>&1
dc logs --no-color > "$OUT/containers.log" 2>&1
python3 "$ROOT/tools/analyze_batman.py" "$OUT" "$CFG"; rc=$?
echo "logs: $OUT"; exit $rc
