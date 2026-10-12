#!/usr/bin/env bash
# Two vehicles: two PX4 x500s, each in its own namespace, each flown by its own flight node over the LOCAL
# DDS domain (loopback). No swarm traffic yet. Checks both arm, take off, fly a square and land.
#   sudo tests/two_vehicles_fly.sh   |   tests/run_rootless.sh tests/two_vehicles_fly.sh
set -u
# under plain sudo HOME may be /root; use the invoking user's home so tool paths resolve
[ -n "${SUDO_USER:-}" ] && HOME="$(getent passwd "$SUDO_USER" | cut -d: -f6)"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
. "$ROOT/tests/_config_flag.sh"          # sets CFG from --config <yaml> (default: two_agent_wifi.yaml)
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
OUT="${OUT:-$(mktemp -d)}"
[ "${COSIM_SANDBOX:-0}" = 1 ] && mount -t tmpfs tmpfs /run/netns
ip link set lo up 2>/dev/null
set +u; source /opt/ros/jazzy/setup.bash; source "$HOME/dev/px4-runtime/ws/install/setup.bash"; source "$ROOT/ros2_ws/install/setup.bash"; set -u
# local domain only: discovery restricted to localhost; no radio-domain config needed here
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=0; unset CYCLONEDDS_URI
fail=0; chk() { if [ "$2" = ok ]; then echo "PASS  $1"; else echo "FAIL  $1  ($2)"; fail=1; fi; }
inns() { local a=$1; shift; ip netns exec ns-$a "$@"; }
echo "logs=$OUT"
python3 "$ROOT/orchestrator/wire.py" up --config "$CFG" >/dev/null || exit 1
python3 "$ROOT/orchestrator/run_stack.py" --config "$CFG" --log-dir "$OUT" > "$OUT/stack.out" 2>&1 &
SP=$!
for i in $(seq 1 60); do grep -qE "ALL READY|NOT READY" "$OUT/stack.out" && break; sleep 2; done
cat "$OUT/stack.out" | grep -E "ready|READY"
grep -q "ALL READY" "$OUT/stack.out" || { chk "PX4 stack ready" "see $OUT/stack.out"; kill -TERM $SP; exit 1; }
sleep 10    # let EKF settle / preflight checks pass
fly() { # name instance
  inns $1 ros2 run px4_offboard_control square_trajectory --ros-args -p px4_instance:=$2 -p side:=4.0 -p altitude:=3.0 -p speed:=2.0 \
      > "$OUT/$1_flight.log" 2>&1 &
}
fly agent1 0; F1=$!; fly agent2 1; F2=$!
wait $F1 $F2
for a in agent1 agent2; do
  grep -q "Reached altitude" "$OUT/${a}_flight.log" && chk "$a took off and reached altitude" ok || chk "$a takeoff" "$(tail -2 "$OUT/${a}_flight.log" | tr '\n' ' ')"
  grep -q "Landed and disarmed" "$OUT/${a}_flight.log" && chk "$a flew the square, landed, disarmed" ok || chk "$a landed" "$(tail -2 "$OUT/${a}_flight.log" | tr '\n' ' ')"
done
kill -TERM $SP; sleep 3
python3 "$ROOT/orchestrator/wire.py" down --config "$CFG" >/dev/null
echo "logs: $OUT"; exit $fail
