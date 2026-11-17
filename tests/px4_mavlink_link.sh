#!/usr/bin/env bash
# Isaac Sim spike, stage A: the pinned PX4 attaches to an external MAVLink simulator, with no Gazebo and no GPU.
#   tests/px4_mavlink_link.sh [--instance N] [--timeout S]      (needs the cosim-px4-runtime image; only the Docker daemon)
# A stub (tools/mavlink_sim_stub.py) stands in for the simulator on a fresh Docker network. PX4 runs with the external-simulator
# airframe (PX4_SIM_MODEL=none_iris) and PX4_SIM_HOST_ADDR set to the stub, the same route Isaac Sim + Pegasus would use.
# Criteria (fixed before the first run; docs/EXPERIMENTS.md, "Isaac Sim stage A"):
#   1. the PX4 log contains "Simulator connected on TCP port <4560+N>."
#   2. the stub reports at least 1 byte received from PX4
#   3. the PX4 environment contains no variable starting PX4_GZ_ (so the Gazebo bridge cannot start)
# Containers and network are named with this script's PID and removed on exit; nothing else is touched.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
INST=0; WAIT=60
while [ $# -gt 0 ]; do
  case "$1" in
    --instance) INST="$2"; shift 2;;
    --timeout) WAIT="$2"; shift 2;;
    *) echo "usage: $0 [--instance N] [--timeout S]"; exit 2;;
  esac
done
IMG=cosim-px4-runtime
BUILD=/opt/PX4-Autopilot/build/px4_sitl_default
PORT=$((4560 + INST))
NET="cosim-stagea-$$"; STUB="$NET-stub"; PX4="$NET-px4"
trap 'docker rm -f "$STUB" "$PX4" >/dev/null 2>&1; docker network rm "$NET" >/dev/null 2>&1' EXIT
trap 'exit 130' INT TERM HUP
fail=0
chk() { if [ "$2" = ok ]; then echo "PASS  $1"; else echo "FAIL  $1  ($2)"; fail=1; fi; }

docker network create "$NET" >/dev/null || { echo "FAIL  cannot create network"; exit 1; }
# the stub listens for WAIT+20 s so it outlives the PX4 wait
docker run -d --name "$STUB" --network "$NET" --entrypoint python3 -v "$ROOT/tools:/stub:ro" "$IMG" \
  /stub/mavlink_sim_stub.py "$PORT" $((WAIT + 20)) >/dev/null 2>&1
sleep 2
if [ "$(docker inspect -f '{{.State.Running}}' "$STUB" 2>/dev/null)" != true ]; then
  chk "stub simulator is running on port $PORT" "$(docker logs "$STUB" 2>&1 | tail -1)"
  echo "RESULT: FAIL"; exit 1
fi
chk "stub simulator is running on port $PORT" ok
STUB_IP=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "$STUB")

# command line and environment as orchestrator/agent_main.py starts PX4 (instance dir as cwd, ROS library path not inherited)
docker run -d --name "$PX4" --network "$NET" --entrypoint bash \
  -e PX4_SIM_MODEL=none_iris -e PX4_SIM_HOST_ADDR="$STUB_IP" -e LD_LIBRARY_PATH=/usr/local/lib "$IMG" \
  -c "mkdir -p $BUILD/instance_$INST && cd $BUILD/instance_$INST && exec $BUILD/bin/px4 -i $INST -d $BUILD/etc" >/dev/null

want="Simulator connected on TCP port $PORT."
for _ in $(seq "$WAIT"); do
  docker logs "$PX4" 2>&1 | grep -qF "$want" && break
  [ "$(docker inspect -f '{{.State.Running}}' "$PX4")" = true ] || break
  sleep 1
done
docker logs "$PX4" 2>&1 | grep -qF "$want" && chk "PX4 log: $want" ok || chk "PX4 log: $want" "not seen in ${WAIT} s; last line: $(docker logs "$PX4" 2>&1 | tail -1)"

sleep 2
rx=$(docker logs "$STUB" 2>&1 | sed -n 's/^stub: received \([0-9]*\) bytes.*/\1/p' | tail -1)
if [ -z "$rx" ]; then                       # the stub prints the total only when its time is up; ask for the running count instead
  docker stop -t 5 "$STUB" >/dev/null 2>&1; rx=$(docker logs "$STUB" 2>&1 | sed -n 's/^stub: received \([0-9]*\) bytes.*/\1/p' | tail -1)
fi
[ "${rx:-0}" -ge 1 ] && chk "stub received bytes from PX4" ok || chk "stub received at least 1 byte from PX4" "got '${rx:-none}'"

gz=$(docker exec "$PX4" env 2>/dev/null | grep -c '^PX4_GZ_')
[ "$gz" = 0 ] && chk "no PX4_GZ_ variable in the PX4 environment" ok || chk "no PX4_GZ_ variable in the PX4 environment" "$gz found"

echo "RESULT: $([ $fail = 0 ] && echo PASS || echo FAIL)"
exit $fail
