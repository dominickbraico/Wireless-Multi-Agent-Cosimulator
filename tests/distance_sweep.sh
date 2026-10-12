#!/usr/bin/env bash
# Distance sweep: distance sweep with real traffic. agent2 is moved along +x (agent1 fixed
# at the origin) while ping, a 2 Mbit/s UDP stream and the leader-follower swarm
# stream run continuously through ns-3. Output: per-step stats, plots, checks.
#   sudo tests/distance_sweep.sh   |   tests/run_rootless.sh tests/distance_sweep.sh
# Env: HOLD (s per step, default 6), STEPS (comma list), OUT (log dir)
set -u
# under plain sudo HOME may be /root; use the invoking user's home so tool paths resolve
[ -n "${SUDO_USER:-}" ] && HOME="$(getent passwd "$SUDO_USER" | cut -d: -f6)"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
. "$ROOT/tests/_config_flag.sh"          # sets CFG from --config <yaml> (default: two_agent_wifi.yaml)
NS3="${NS3:-$HOME/dev/ns-3-build/ns-allinone-3.48/ns-3.48/build/scratch/wifi_adhoc_tap/ns3.48-wifi_adhoc_tap-optimized}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
OUT="${OUT:-$(mktemp -d)}"; HOLD=${HOLD:-6}
STEPS="${STEPS:-1,10,20,30,40,45,48,50,51,52,53,55,60,70,80,100,150,200,100,70,55,52,51,50,45,10}"   # dense around the predicted cliff
NSTEPS=$(echo "$STEPS" | tr ',' '\n' | wc -l)
[ "${COSIM_SANDBOX:-0}" = 1 ] && mount -t tmpfs tmpfs /run/netns
ip link set lo up 2>/dev/null
set +u; source /opt/ros/jazzy/setup.bash; source "$ROOT/ros2_ws/install/setup.bash"; set -u
[ "$RMW_IMPLEMENTATION" = rmw_cyclonedds_cpp ] && export CYCLONEDDS_URI="file://$ROOT/config/dds/cyclonedds_radio.xml"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export PATH_ORIG="$PATH"
GZPATH=/usr/bin:$PATH
WORLD=$(python3 -c "import yaml;print(yaml.safe_load(open('$CFG'))['simulator']['world_name'])")
inns() { local a=$1; shift; ip netns exec ns-$a "$@"; }
TOTAL=$(( NSTEPS * HOLD + 8 ))        # traffic/ns-3 duration (s) after the sweep starts
python3 "$ROOT/tools/link_budget.py" "$CFG" --out "$OUT/prediction.json" > "$OUT/prediction.txt" || exit 1
cat "$OUT/prediction.txt" | tr -d '\n' ; echo
echo "steps=$NSTEPS hold=${HOLD}s total~${TOTAL}s logs=$OUT RMW=$RMW_IMPLEMENTATION"

python3 "$ROOT/orchestrator/wire.py" up --config "$CFG" >/dev/null || exit 1
( export PATH=$GZPATH GZ_CONFIG_PATH=/usr/share/gz GZ_PARTITION=cosim GZ_IP=127.0.0.1
  exec /usr/bin/gz sim -s -r --headless-rendering "$ROOT/worlds/$WORLD.sdf" ) > "$OUT/gz.log" 2>&1 &
GZ=$!; sleep 6
"$NS3" --config="$CFG" --log-dir="$OUT/ns3" --duration=$((TOTAL+14)) > "$OUT/ns3.out" 2>&1 &
NSPID=$!; sleep 2
( export GZ_PARTITION=cosim GZ_IP=127.0.0.1 PATH=$GZPATH GZ_CONFIG_PATH=/usr/share/gz
  exec python3 "$ROOT/adapters/gazebo/pose_bridge.py" --config "$CFG" --log-dir "$OUT/bridge" --duration $((TOTAL+25)) ) > "$OUT/bridge.out" 2>&1 &
PB=$!; sleep 3
# warm the sweep client first (its transport join stalls the sim once), then start traffic
( export GZ_PARTITION=cosim GZ_IP=127.0.0.1 PATH=$GZPATH GZ_CONFIG_PATH=/usr/share/gz
  exec python3 "$ROOT/adapters/gazebo/sweep.py" --world "$WORLD" --agent agent2 --y-agent "" --x "$STEPS" \
       --hold $HOLD --warmup 4 --out "$OUT/steps.json" ) > "$OUT/sweep.out" 2>&1 &
SW=$!
sleep 2    # sweep is in its warmup; agent2 sits at 10 m (link good)

DUR=$((TOTAL))
# agent1 (fixed at origin) is the sender/leader; agent2 (moves) is the receiver/follower
inns agent2 python3 "$ROOT/tools/udp_probe.py" rx 10.10.0.2 5001 $((DUR+4)) "$OUT/udp_rx.csv" &
inns agent2 ros2 run swarm_control swarm_node --ros-args -p role:=follower -p agent_id:=2 \
     -p log_csv:="$OUT/swarm_follower.csv" > "$OUT/swarm_follower.log" 2>&1 &
sleep 2
inns agent1 ros2 run swarm_control swarm_node --ros-args -p role:=leader -p agent_id:=1 -p rate_hz:=10.0 \
     > "$OUT/swarm_leader.log" 2>&1 &
inns agent1 python3 "$ROOT/tools/udp_probe.py" tx 10.10.0.2 5001 2000000 1000 $DUR "$OUT/udp_tx.csv" &
inns agent1 timeout $DUR ping -D -O -i 0.2 -W 1 10.10.0.2 > "$OUT/ping.txt" 2>&1 &

wait $SW
sleep 5
pkill -f "swarm_node" 2>/dev/null
wait $PB 2>/dev/null
wait $NSPID 2>/dev/null          # let ns-3 finish normally so summary.json is written
kill $GZ 2>/dev/null; wait 2>/dev/null
python3 "$ROOT/orchestrator/wire.py" down --config "$CFG" >/dev/null
python3 "$ROOT/tools/analyze_sweep.py" "$OUT" "$CFG" "$OUT/prediction.json"; rc=$?
echo "logs: $OUT"; exit $rc
