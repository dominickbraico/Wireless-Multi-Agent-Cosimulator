#!/usr/bin/env bash
# Leader-follower with N PX4 x500s (N = number of agents in the experiment YAML), ALL robot-to-robot traffic through ns-3.
#   leader:    flies a square; its swarm node reads PX4 state (local domain) and sends LeaderState on the radio domain
#   followers: each swarm node receives over the emulated link and hands a target to that agent's follower flight node (local domain)
# Roles and per-follower formation offsets come from the YAML (orchestrator/fleet.py): by default the first agent leads and
# each follower holds the offset it was spawned at.
# Ends by stopping the leader's swarm node after it lands: every follower must detect link loss and land.
#   sudo tests/leader_follower_fly.sh [--config YAML]   |   tests/run_rootless.sh tests/leader_follower_fly.sh [--config YAML]
# Env: PX4_WORLD (worlds/<name>.sdf, default from the YAML: px4_flat), CFG, OUT, OFFSET (x,y,z world frame: force this offset on
#      every follower instead of the YAML-derived ones), SIDE (m), ALT (m), SPEED (m/s)
set -u
# under plain sudo HOME may be /root; use the invoking user's home so tool paths resolve
[ -n "${SUDO_USER:-}" ] && HOME="$(getent passwd "$SUDO_USER" | cut -d: -f6)"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
. "$ROOT/tests/_config_flag.sh"          # sets CFG from --config <yaml> (default: two_agent_wifi.yaml)
NS3="${NS3:-$HOME/dev/ns-3-build/ns-allinone-3.48/ns-3.48/build/scratch/wifi_adhoc_tap/ns3.48-wifi_adhoc_tap-optimized}"
OUT="${OUT:-$(mktemp -d)}"; OFFSET="${OFFSET:-}"; SIDE="${SIDE:-10.0}"; ALT="${ALT:-5.0}"; SPEED="${SPEED:-1.5}"
[ "${COSIM_SANDBOX:-0}" = 1 ] && mount -t tmpfs tmpfs /run/netns
ip link set lo up 2>/dev/null
set +u; source /opt/ros/jazzy/setup.bash; source "$HOME/dev/px4-runtime/ws/install/setup.bash"; source "$ROOT/ros2_ws/install/setup.bash"; set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
inns() { local a=$1; shift; ip netns exec ns-$a "$@"; }
# Reaper: whatever happens (failure, Ctrl-C, a supervisor that died), nothing from this run may outlive it.
descendants() { local p; for p in $(pgrep -P "$1" 2>/dev/null); do echo "$p"; descendants "$p"; done; }
reap() {
  trap - EXIT
  local pids; pids="$(descendants $$) $(cat "$OUT/stack.pids" 2>/dev/null)"
  kill -TERM $pids 2>/dev/null; sleep 2; kill -KILL $pids 2>/dev/null
}
trap 'rc=$?; reap; exit $rc' EXIT
trap 'exit 130' INT TERM HUP     # make Ctrl-C / kill run the EXIT trap (and so the reaper)
# who leads / follows and the formation offsets (orchestrator/fleet.py): role name instance spawn_x spawn_y off_x off_y off_z
FLEET=$(python3 "$ROOT/orchestrator/fleet.py" "$CFG" ${OFFSET:+--offset "$OFFSET"}) || exit 2
LEADER=""; LSPAWN=""; LINST=""; FOLLOWERS=()
while IFS=$'\t' read -r role name inst sx sy ox oy oz; do
  if [ "$role" = leader ]; then LEADER=$name; LINST=$inst; LSPAWN="[$sx, $sy]"
  else FOLLOWERS+=("$name $inst [$sx,$sy] [$ox,$oy,$oz]"); fi
done <<< "$FLEET"
NAGENTS=$(( ${#FOLLOWERS[@]} + 1 ))
echo "logs=$OUT leader=$LEADER followers=${#FOLLOWERS[@]} square=${SIDE}m alt=${ALT}m speed=${SPEED}m/s"
printf '  %s\n' "${FOLLOWERS[@]}"

# the flight stack runs with the radio firewall ON (config copy; the original YAML is untouched)
CFGRUN="$OUT/experiment_run.yaml"
python3 - "$CFG" "$CFGRUN" <<'PY'
import sys, yaml
import os
c = yaml.safe_load(open(sys.argv[1])); c["network"]["radio_firewall"]["enabled"] = True
if os.environ.get("PX4_WORLD"): c["simulator"]["px4_world_name"] = os.environ["PX4_WORLD"]   # e.g. baylands (worlds/<name>.sdf)
yaml.safe_dump(c, open(sys.argv[2], "w"), sort_keys=False)
PY
CFG="$CFGRUN"
WORLDNAME=$(python3 -c "import yaml;print(yaml.safe_load(open('$CFG'))['simulator']['px4_world_name'])")
echo "world: $WORLDNAME"
python3 "$ROOT/orchestrator/wire.py" up --config "$CFG" >/dev/null || exit 1
python3 "$ROOT/orchestrator/run_stack.py" --config "$CFG" --log-dir "$OUT" --ready-timeout "$((90 + 30 * NAGENTS))" > "$OUT/stack.out" 2>&1 &
SP=$!
READY_S=$((90 + 30 * NAGENTS))     # PX4 start-up time grows with the number of vehicles
for i in $(seq 1 $((READY_S / 2))); do grep -qE "ALL READY|NOT READY" "$OUT/stack.out" && break; sleep 2; done
grep -q "ALL READY" "$OUT/stack.out" || { echo "FAIL  PX4 stack not ready (see $OUT/stack.out)"; kill -TERM $SP; exit 1; }

# ns-3 (real time) + pose bridge (Gazebo truth -> ns-3 mobility). PATH/GZ env: the real `gz` must come first on PATH (see docs/SETUP.md).
"$NS3" --config="$CFG" --log-dir="$OUT/ns3" --duration=400 > "$OUT/ns3.out" 2>&1 &
NSPID=$!; sleep 2
( export GZ_PARTITION=cosim GZ_IP=10.20.0.1 PATH=/usr/bin:$PATH GZ_CONFIG_PATH=/usr/share/gz
  exec python3 "$ROOT/adapters/gazebo/pose_bridge.py" --config "$CFG" --log-dir "$OUT/bridge" --world "$WORLDNAME" --px4-models ) > "$OUT/bridge.out" 2>&1 &
PB=$!; sleep 8      # EKF settle + first poses

# Optional: pause so a Gazebo GUI can be attached before anything flies.   WAIT_FOR_GUI=1  ->  touch "$OUT/go" to start
if [ "${WAIT_FOR_GUI:-0}" = 1 ]; then
  echo "=== Both PX4 vehicles are up and waiting on the ground. Open the Gazebo GUI in ANOTHER terminal (not the ROS-sourced one):"
  echo "    GZ_PARTITION=cosim GZ_IP=10.20.0.1 PATH=/usr/bin:\$PATH GZ_CONFIG_PATH=/usr/share/gz \\"
  echo "    GZ_SIM_RESOURCE_PATH=\$HOME/.simulation-gazebo/models:\$HOME/.simulation-gazebo/worlds  /usr/bin/gz sim -g"
  echo "=== then start the flight with:   touch \"$OUT/go\"      (auto-starts after 300 s)"
  for i in $(seq 1 300); do [ -e "$OUT/go" ] && break; sleep 1; done
fi

# swarm nodes (px4 mode: local domain 0 localhost-only + radio domain 42 on radio0)
swarm() { # agent role instance spawn_xy extra...
  local a=$1 role=$2 inst=$3 spawn=$4; shift 4
  inns $a env SWARM_MODE=px4 CYCLONEDDS_URI="file://$ROOT/config/dds/cyclonedds_agent.xml" \
      ros2 run swarm_control swarm_node --ros-args -p role:=$role -p agent_id:=$((inst+1)) -p px4_instance:=$inst \
      -p spawn_xy:="$spawn" "$@" > "$OUT/swarm_$a.log" 2>&1 &
}
for f in "${FOLLOWERS[@]}"; do
  read -r name inst spawn off <<< "$f"
  swarm $name follower $inst "$spawn" -p offset:="$off" -p log_csv:="$OUT/swarm_follower_$name.csv"
done
swarm $LEADER leader $LINST "$LSPAWN" -p rate_hz:=10.0
sleep 3

# flight nodes (local domain only)
LOC="ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=0"
FFS=""
for f in "${FOLLOWERS[@]}"; do
  read -r name inst spawn off <<< "$f"
  inns $name env $LOC ros2 run px4_offboard_control follower_flight --ros-args -p px4_instance:=$inst -p altitude:=$ALT \
      -p spawn_xy:="$spawn" -p on_link_lost:=land -p link_timeout_s:=2.5 > "$OUT/${name}_flight.log" 2>&1 &
  FFS="$FFS $!"
done
inns $LEADER env $LOC ros2 run px4_offboard_control square_trajectory --ros-args -p px4_instance:=$LINST -p side:=$SIDE -p altitude:=$ALT \
    -p speed:=$SPEED > "$OUT/${LEADER}_flight.log" 2>&1 &
LF=$!
# the first follower's link log also under the old name (demo/monitor.py reads swarm_follower.csv)
read -r first _ <<< "${FOLLOWERS[0]}"; ln -sf "swarm_follower_$first.csv" "$OUT/swarm_follower.csv"

# leader finishes -> stop its swarm node -> follower must see the link go away and land
for i in $(seq 1 200); do grep -q "Landed and disarmed" "$OUT/${LEADER}_flight.log" && break; sleep 1; done
sleep 2
# `ros2 run` is only a launcher: TERM-ing it leaves the node running, so match the node process itself
pkill -TERM -f "swarm_node --ros-args -p role:=leader" 2>/dev/null
for i in $(seq 1 60); do
  left=0; for f in "${FOLLOWERS[@]}"; do read -r name _ <<< "$f"; grep -q "Landed and disarmed" "$OUT/${name}_flight.log" || left=1; done
  [ $left = 0 ] && break; sleep 1; done
sleep 2
pkill -TERM -f "swarm_node --ros-args -p role:=follower" 2>/dev/null; kill -TERM $FFS $PB 2>/dev/null; sleep 1
kill -TERM $NSPID 2>/dev/null; for i in $(seq 1 20); do kill -0 $NSPID 2>/dev/null || break; sleep 1; done
# what the radio firewall had to block (attempted leaks) before tearing the namespaces down
for a in $LEADER $(for f in "${FOLLOWERS[@]}"; do read -r n _ <<< "$f"; echo $n; done); do echo "firewall $a: $(inns $a nft list counters table inet cosim_radio 2>/dev/null | tr -s ' \n' ' ' | sed 's/counter //g')"; done | tee "$OUT/firewall_counters.txt"
kill -TERM $SP 2>/dev/null; sleep 3
python3 "$ROOT/orchestrator/wire.py" down --config "$CFG" >/dev/null
python3 "$ROOT/tools/analyze_formation.py" "$OUT" "$CFG" ${OFFSET:+--offset "$OFFSET"}; rc=$?
echo "logs: $OUT"; exit $rc
