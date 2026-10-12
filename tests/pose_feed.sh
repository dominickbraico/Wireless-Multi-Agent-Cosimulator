#!/usr/bin/env bash
# Pose-feed test: simulator -> ns-3 pose feed, in isolation (no robot traffic).
# Gazebo server (headless) + pose bridge + ns-3 listener; agent2 is swept 1..200 m
# with the set_pose service; ns-3's applied positions are compared with commands.
#   sudo tests/pose_feed.sh   |   tests/run_rootless.sh tests/pose_feed.sh
set -u
# under plain sudo HOME may be /root; use the invoking user's home so tool paths resolve
[ -n "${SUDO_USER:-}" ] && HOME="$(getent passwd "$SUDO_USER" | cut -d: -f6)"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
. "$ROOT/tests/_config_flag.sh"          # sets CFG from --config <yaml> (default: two_agent_wifi.yaml)
NS3="${NS3:-$HOME/dev/ns-3-build/ns-allinone-3.48/ns-3.48/build/scratch/wifi_adhoc_tap/ns3.48-wifi_adhoc_tap-optimized}"
OUT="${OUT:-$(mktemp -d)}"
HOLD=${HOLD:-2}
STEPS=(1 5 20 50 100 150 200)
[ "${COSIM_SANDBOX:-0}" = 1 ] && mount -t tmpfs tmpfs /run/netns
ip link set lo up 2>/dev/null
# ROS 2's gz stub shadows the real binary; pin the real Gazebo.
export PATH=/usr/bin:$PATH GZ_CONFIG_PATH=/usr/share/gz GZ_PARTITION=cosim GZ_IP=127.0.0.1
WORLD=$(python3 -c "import yaml;print(yaml.safe_load(open('$CFG'))['simulator']['world_name'])")

python3 "$ROOT/orchestrator/wire.py" up --config "$CFG" >/dev/null || exit 1   # TAPs must exist for ns-3
DUR=$(( ${#STEPS[@]} * HOLD + 20 ))
/usr/bin/gz sim -s -r --headless-rendering "$ROOT/worlds/$WORLD.sdf" > "$OUT/gz.log" 2>&1 &
GZ=$!; sleep 6
"$NS3" --config="$CFG" --log-dir="$OUT/ns3" --duration=$DUR > "$OUT/ns3.out" 2>&1 &
NSPID=$!; sleep 2
python3 "$ROOT/adapters/gazebo/pose_bridge.py" --config "$CFG" --log-dir "$OUT/bridge" --duration $((DUR-4)) > "$OUT/bridge.out" 2>&1 &
PB=$!; sleep 3
python3 "$ROOT/adapters/gazebo/sweep.py" --world "$WORLD" --agent agent2 --y-agent agent1 \
  --x "$(IFS=,; echo "${STEPS[*]}")" --hold $HOLD --out "$OUT/steps.json" || { echo "sweep failed"; }
wait $PB; wait $NSPID 2>/dev/null; kill $GZ 2>/dev/null
python3 "$ROOT/tools/check_pose_feed.py" "$OUT/ns3" "$OUT/bridge" "$OUT/steps.json" "$CFG"; rc=$?
cat "$OUT/bridge.out"
python3 "$ROOT/orchestrator/wire.py" down --config "$CFG" >/dev/null
echo "logs: $OUT"; exit $rc
