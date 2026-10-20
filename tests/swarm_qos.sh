#!/usr/bin/env bash
# Checks that the LeaderState QoS parameters reach the DDS endpoints and have the expected effect.
# No network emulation: leader and follower talk over loopback on their own DDS domain.
#   1. matching profiles (best_effort/volatile/1, reliable/transient_local/3): the follower receives messages (>= 40 of the ~80 sent)
#   2. mismatched profiles (best_effort publisher, reliable subscriber): DDS must NOT match them, nothing is received
#   3. invalid values are rejected before anything starts
# Needs the workspace built and sourced:  (cd ros2_ws && colcon build --symlink-install) && source ros2_ws/install/setup.bash
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT=${OUT:-$ROOT/logs/swarm_qos_$(date +%Y%m%d_%H%M%S)}; mkdir -p "$OUT"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
FAIL=0
check() { if [ "$2" = ok ]; then echo "PASS  $1"; else echo "FAIL  $1  ($3)"; FAIL=1; fi; }

# run_pair <tag> <leader rel> <leader dur> <leader depth> <follower rel> <follower dur> <follower depth>
run_pair() {
  local t=$1
  setsid timeout -s INT -k 3 10 ros2 run swarm_control swarm_node --ros-args -p role:=follower -p agent_id:=2 -p log_csv:="$OUT/$t.csv" \
      -p qos_reliability:=$5 -p qos_durability:=$6 -p qos_depth:=$7 > "$OUT/$t.follower.log" 2>&1 &
  local f=$!; sleep 2
  setsid timeout -s INT -k 3 8 ros2 run swarm_control swarm_node --ros-args -p role:=leader -p agent_id:=1 -p rate_hz:=10.0 \
      -p qos_reliability:=$2 -p qos_durability:=$3 -p qos_depth:=$4 > "$OUT/$t.leader.log" 2>&1 &
  local l=$!; wait $l $f 2>/dev/null
  kill -TERM -- -$l -$f 2>/dev/null     # ros2 run leaves its child node behind; each pair ran in its own session
  echo $(( $(wc -l < "$OUT/$t.csv" 2>/dev/null || echo 1) - 1 ))
}

n=$(run_pair be best_effort volatile 1 best_effort volatile 1)
[ "$n" -ge 40 ] && check "best_effort/volatile/1 delivers ($n msgs in ~6 s at 10 Hz)" ok || check "best_effort/volatile/1 delivers" bad "got $n"
grep -q "BEST_EFFORT VOLATILE KEEP_LAST depth=1" "$OUT/be.leader.log" && check "effective QoS logged (best_effort)" ok || check "effective QoS logged (best_effort)" bad "$OUT/be.leader.log"

n=$(run_pair rel reliable transient_local 3 reliable transient_local 3)
[ "$n" -ge 40 ] && check "reliable/transient_local/3 delivers ($n msgs)" ok || check "reliable/transient_local/3 delivers" bad "got $n"
grep -q "RELIABLE TRANSIENT_LOCAL KEEP_LAST depth=3" "$OUT/rel.leader.log" && check "effective QoS logged (reliable)" ok || check "effective QoS logged (reliable)" bad "$OUT/rel.leader.log"

n=$(run_pair mis best_effort volatile 1 reliable volatile 1)
[ "$n" -eq 0 ] && check "best_effort publisher does not match a reliable subscriber (0 msgs)" ok || check "mismatch yields no data" bad "got $n"

timeout 15 ros2 run swarm_control swarm_node --ros-args -p qos_reliability:=bogus > "$OUT/bad.log" 2>&1
grep -q "qos: reliability in" "$OUT/bad.log" && check "invalid reliability rejected" ok || check "invalid reliability rejected" bad "$OUT/bad.log"

[ $FAIL = 0 ] && echo "ALL PASS" || echo "SOME FAILED (logs: $OUT)"
exit $FAIL
