#!/usr/bin/env bash
# Rover drive test in Docker Compose: a simulator container, an ns-3 container and one container per rover (PX4 SITL with the rover airframe + ROS 2).
# Every rover drives a square from its spawn point and disarms; tools/analyze_rover.py checks PX4's position and the Gazebo ground truth.
#   tests/containers_rover.sh [--config YAML]      (images first: orchestrator/cosim.py build; default config: rover_two_agent.yaml)
# Env: OUT (log directory), PX4_WORLD (worlds/<name>.sdf, default from the YAML)
# Needs only the Docker daemon. No Gazebo GUI. The radio is not exercised (no swarm traffic in this scenario); see docs/EXPERIMENTS.md.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CFG="${CFG:-$ROOT/config/experiments/rover_two_agent.yaml}"
. "$ROOT/tests/_config_flag.sh"          # sets CFG from --config <yaml>
OUT="${OUT:-$(mktemp -d)}"; mkdir -p "$OUT"
cosim() { python3 "$ROOT/orchestrator/cosim.py" "$@"; }
dc() { docker compose -f "$OUT/compose/docker-compose.yml" "$@"; }   # $ROOT may contain spaces
trap 'cosim down --out "$OUT" >/dev/null 2>&1' EXIT
trap 'exit 130' INT TERM HUP
AGENTS=$(python3 -c "import yaml,sys; print(' '.join(a['name'] for a in yaml.safe_load(open(sys.argv[1]))['agents']))" "$CFG")
N=$(echo $AGENTS | wc -w)
echo "logs=$OUT rovers=$N world=${PX4_WORLD:-from YAML}"

cosim up --config "$CFG" --out "$OUT" --mode flight ${PX4_WORLD:+--world "$PX4_WORLD"} --timeout "$((150 + 30 * N))" > "$OUT/up.log" 2>&1 \
  || { tail -40 "$OUT/up.log"; echo "FAIL  stack not healthy (see $OUT/up.log)"; exit 1; }
echo "stack up: simulator, ns3 and $N rover containers healthy"
sleep 8                                   # EKF settle + first poses reach ns-3
touch "$OUT/go"                           # every agent supervisor waits for this file

for i in $(seq 1 400); do
  left=0; for a in $AGENTS; do [ -e "$OUT/$a.done" ] || left=1; done
  [ $left = 0 ] && break; sleep 1
done
[ $left = 0 ] || echo "WARN  not every rover finished within 400 s"
sleep 2
dc stop ns3 >/dev/null 2>&1               # SIGTERM: ns-3 writes its summary
dc stop simulator >/dev/null 2>&1
dc logs --no-color > "$OUT/containers.log" 2>&1
python3 "$ROOT/tools/analyze_rover.py" "$OUT" "$OUT/compose/experiment_run.yaml"; rc=$?
echo "logs: $OUT"; exit $rc
