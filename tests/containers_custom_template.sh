#!/usr/bin/env bash
# Custom-controller scenario in Docker Compose with the template controller (examples/template_ws, altitude consensus):
# a simulator container, an ns-3 container and one container per agent (PX4 SITL, the radio relay, the user's launch file).
#   tests/containers_custom_template.sh [--config YAML]      (default: custom_consensus.yaml)
# Images first:  python3 orchestrator/cosim.py build --config config/experiments/custom_consensus.yaml
# Env: CONTROL=1      control case: the relay carries a topic the controller does not use, so the altitudes must NOT converge
#      BROKEN=launch  the launch file does not exist in the image: every agent must report "launch failed" and still finish
#      BROKEN=type    a radio topic's message type does not exist: every agent must report "relay failed" before the go signal
#      PX4_WORLD (worlds/<name>.sdf, default from the YAML), OUT (log directory)
# Pass criteria: docs/EXPERIMENTS.md, "Custom controller template". Needs only the Docker daemon.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CFG="$ROOT/config/experiments/custom_consensus.yaml"
. "$ROOT/tests/_config_flag.sh"          # sets CFG from --config <yaml>
OUT="${OUT:-$ROOT/logs/custom_template_$(date +%Y%m%d_%H%M%S)}"; mkdir -p "$OUT"
cosim() { python3 "$ROOT/orchestrator/cosim.py" "$@"; }
dc() { docker compose -f "$OUT/compose/docker-compose.yml" "$@"; }   # $ROOT may contain spaces
trap 'cosim down --out "$OUT" >/dev/null 2>&1' EXIT
trap 'exit 130' INT TERM HUP
FAIL=0
check() { if [ "$2" = ok ]; then echo "PASS  $1"; else echo "FAIL  $1  ($3)"; FAIL=1; fi; }

VARIANT="${BROKEN:+broken-$BROKEN}"; [ "${CONTROL:-0}" = 1 ] && VARIANT=control
if [ -n "$VARIANT" ]; then      # a modified copy of the YAML; the image is the one built for the original (same experiment name)
  python3 - "$CFG" "$OUT/experiment_variant.yaml" "$VARIANT" "$ROOT" <<'PY' || exit 2
import os, sys, yaml
c = yaml.safe_load(open(sys.argv[1])); x = c["scenario"]["custom"]; v = sys.argv[3]
x["workspace"] = x["workspace"] if os.path.isabs(x["workspace"]) else os.path.join(sys.argv[4], x["workspace"])
if v == "control":
    x["radio_topics"][0]["name"] = "/swarm/unused"
elif v == "broken-type":
    x["radio_topics"][0]["type"] = "consensus_msgs/msg/NoSuchMessage"
yaml.safe_dump(c, open(sys.argv[2], "w"), sort_keys=False)
PY
  CFG="$OUT/experiment_variant.yaml"
fi
AGENTS=$(python3 -c "import yaml,sys; print(' '.join(a['name'] for a in yaml.safe_load(open(sys.argv[1]))['agents']))" "$CFG")
N=$(echo $AGENTS | wc -w)
echo "logs=$OUT agents=$N variant=${VARIANT:-none}"

cosim up --config "$CFG" --out "$OUT" --mode flight ${PX4_WORLD:+--world "$PX4_WORLD"} --timeout "$((150 + 30 * N))" > "$OUT/up.log" 2>&1; up=$?
# broken-type: the agents stop on purpose once their relay refuses the type, so "up" may report an exited container
[ $up = 0 ] || [ "$VARIANT" = broken-type ] || { tail -40 "$OUT/up.log"; echo "FAIL  stack not healthy (see $OUT/up.log)"; exit 1; }
echo "stack up (exit $up): simulator, ns3 and $N agent containers"
if [ "$VARIANT" = broken-launch ]; then   # the launch file is validated on the host, so it is removed inside the running containers instead
  for a in $AGENTS; do dc exec -T "$a" bash -c 'rm -f /opt/user_ws/install/*/share/*/launch/*.launch.py' ; done
fi
sleep 8                                   # EKF settle + first poses reach ns-3

if [ "$VARIANT" = broken-type ]; then
  ok=1; for a in $AGENTS; do dc logs --no-color "$a" 2>/dev/null | grep -q "agent_main\[$a\]: relay failed: .*NoSuchMessage" || ok=0; done
  dc logs --no-color > "$OUT/containers.log" 2>&1
  [ $ok = 1 ] && check "unknown message type: every agent reports 'relay failed' naming the type, before the go signal" ok || check "unknown message type reported" bad "$OUT/containers.log"
  echo "logs: $OUT"; exit $FAIL
fi

touch "$OUT/go"                           # every agent supervisor waits for this file
for i in $(seq 1 300); do
  left=0; for a in $AGENTS; do [ -e "$OUT/$a.done" ] || left=1; done
  [ $left = 0 ] && break; sleep 1
done
[ $left = 0 ] || echo "WARN  not every agent finished within 300 s"
sleep 2
for a in $AGENTS; do echo "firewall $a: $(dc exec -T "$a" nft list counters table inet cosim_radio 2>/dev/null | tr -s ' \n' ' ' | sed 's/counter //g')"; done | tee "$OUT/firewall_counters.txt"
for a in $AGENTS; do echo "mgmt-isolation $a: $(dc exec -T "$a" nft list counters table inet cosim_mgmt 2>/dev/null | tr -s ' \n' ' ' | sed 's/counter //g')"; done | tee "$OUT/mgmt_firewall_counters.txt"
dc stop ns3 >/dev/null 2>&1               # SIGTERM: ns-3 writes its summary
dc stop simulator >/dev/null 2>&1
dc logs --no-color > "$OUT/containers.log" 2>&1

if [ "$VARIANT" = broken-launch ]; then
  ok=1; for a in $AGENTS; do grep -q "agent_main\[$a\]: launch failed, see ${a}_launch.log" "$OUT/containers.log" && [ -e "$OUT/$a.done" ] || ok=0; done
  [ $ok = 1 ] && check "missing launch file: every agent reports 'launch failed' and still finishes" ok || check "missing launch file reported" bad "$OUT/containers.log"
  echo "logs: $OUT"; exit $FAIL
fi
if [ "$VARIANT" = control ]; then
  python3 "$ROOT/tools/analyze_custom.py" "$OUT" "$CFG" --consensus --expect-no-consensus; rc=$?
else
  python3 "$ROOT/tools/analyze_custom.py" "$OUT" "$CFG" --consensus --min-delivery 0.95; rc=$?
fi
echo "logs: $OUT"; exit $rc
