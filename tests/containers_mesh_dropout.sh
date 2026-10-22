#!/usr/bin/env bash
# Mesh-dropout scenario in Docker Compose: a simulator container, an ns-3 container and one container per agent (PX4 SITL + ROS 2).
# One agent wanders (seeded random waypoints plus an excursion that drops 1 or 2 links), the others hover; every agent broadcasts a heartbeat
# on the radio. Afterwards: did the planned links drop and recover, in the model and in the agents' own heartbeats? (docs/MESH_DROPOUT.md)
#   tests/containers_mesh_dropout.sh [--config YAML]      (default: four_agent_mesh_dropout.yaml; images first: orchestrator/cosim.py build)
# Env: MESH_SEED (override scenario.mesh_dropout.seed for this run), WAIT_FOR_GUI=1 (hold the flight until $OUT/go exists),
#      PX4_WORLD (worlds/<name>.sdf, default from the YAML), OUT (log directory)
# Needs only the Docker daemon (no sudo, no host network changes). The Gazebo GUI is not part of the stack.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CFG="$ROOT/config/experiments/four_agent_mesh_dropout.yaml"
. "$ROOT/tests/_config_flag.sh"          # sets CFG from --config <yaml>
OUT="${OUT:-$(mktemp -d)}"; mkdir -p "$OUT"
cosim() { python3 "$ROOT/orchestrator/cosim.py" "$@"; }
dc() { docker compose -f "$OUT/compose/docker-compose.yml" "$@"; }   # $ROOT may contain spaces
trap 'cosim down --out "$OUT" >/dev/null 2>&1' EXIT
trap 'exit 130' INT TERM HUP
if [ -n "${MESH_SEED:-}" ]; then
  python3 - "$CFG" "$OUT/experiment_seeded.yaml" "$MESH_SEED" <<'PY' || exit 2
import sys, yaml
c = yaml.safe_load(open(sys.argv[1])); c.setdefault("scenario", {}).setdefault("mesh_dropout", {})["seed"] = int(sys.argv[3])
yaml.safe_dump(c, open(sys.argv[2], "w"), sort_keys=False)
PY
  CFG="$OUT/experiment_seeded.yaml"
fi
python3 "$ROOT/orchestrator/mesh_plan.py" "$CFG" > "$OUT/plan_preview.json" || { echo "FAIL  the YAML does not give a valid plan"; exit 2; }
AGENTS=$(python3 -c "import yaml,sys; print(' '.join(a['name'] for a in yaml.safe_load(open(sys.argv[1]))['agents']))" "$CFG")
N=$(echo $AGENTS | wc -w)
echo "logs=$OUT agents=$N world=${PX4_WORLD:-from YAML} seed=${MESH_SEED:-from YAML}"

cosim up --config "$CFG" --out "$OUT" --mode flight ${PX4_WORLD:+--world "$PX4_WORLD"} --timeout "$((150 + 30 * N))" > "$OUT/up.log" 2>&1 \
  || { tail -40 "$OUT/up.log"; echo "FAIL  stack not healthy (see $OUT/up.log)"; exit 1; }
echo "stack up: simulator, ns3 and $N agent containers healthy"
sleep 8                                   # EKF settle + first poses reach ns-3
if [ "${WAIT_FOR_GUI:-0}" = 1 ]; then
  echo "=== All vehicles are up and waiting on the ground. Start the flight with:   touch \"$OUT/go\"      (auto-starts after 300 s)"
  for i in $(seq 1 300); do [ -e "$OUT/go" ] && break; sleep 1; done
fi
touch "$OUT/go"                           # every agent supervisor waits for this file

for i in $(seq 1 900); do
  left=0; for a in $AGENTS; do [ -e "$OUT/$a.done" ] || left=1; done
  [ $left = 0 ] && break; sleep 1
done
[ $left = 0 ] || echo "WARN  not every agent finished within 900 s"
sleep 2
for a in $AGENTS; do echo "firewall $a: $(dc exec -T "$a" nft list counters table inet cosim_radio 2>/dev/null | tr -s ' \n' ' ' | sed 's/counter //g')"; done | tee "$OUT/firewall_counters.txt"
for a in $AGENTS; do echo "mgmt-isolation $a: $(dc exec -T "$a" nft list counters table inet cosim_mgmt 2>/dev/null | tr -s ' \n' ' ' | sed 's/counter //g')"; done | tee "$OUT/mgmt_firewall_counters.txt"
dc stop ns3 >/dev/null 2>&1               # SIGTERM: ns-3 writes its summary
dc stop simulator >/dev/null 2>&1
dc logs --no-color > "$OUT/containers.log" 2>&1
python3 "$ROOT/tools/analyze_mesh_dropout.py" "$OUT" "$CFG"; rc=$?
echo "logs: $OUT"; exit $rc
