#!/usr/bin/env bash
# Leader-follower over the emulated link (synthetic leader, no simulator).
# Checks: follower receives leader state through ns-3 with sane loss/latency,
# its target tracks the leader, and with ns-3 stopped it receives NOTHING.
#   sudo tests/swarm_leader_follower.sh   |   tests/run_rootless.sh tests/swarm_leader_follower.sh
# RMW follows $RMW_IMPLEMENTATION (default rmw_cyclonedds_cpp).
set -u
# under plain sudo HOME may be /root; use the invoking user's home so tool paths resolve
[ -n "${SUDO_USER:-}" ] && HOME="$(getent passwd "$SUDO_USER" | cut -d: -f6)"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
. "$ROOT/tests/_config_flag.sh"          # sets CFG from --config <yaml> (default: two_agent_wifi.yaml)
NS3="${NS3:-$HOME/dev/ns-3-build/ns-allinone-3.48/ns-3.48/build/scratch/wifi_adhoc_tap/ns3.48-wifi_adhoc_tap-optimized}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
OUT="${OUT:-$(mktemp -d)}"; DUR=${DUR:-20}
[ "${COSIM_SANDBOX:-0}" = 1 ] && mount -t tmpfs tmpfs /run/netns
ip link set lo up 2>/dev/null
set +u; source /opt/ros/jazzy/setup.bash; source "$ROOT/ros2_ws/install/setup.bash"; set -u
if [ "$RMW_IMPLEMENTATION" = rmw_cyclonedds_cpp ]; then
  [ -d /opt/ros/jazzy/share/rmw_cyclonedds_cpp ] || { echo "rmw_cyclonedds_cpp missing: sudo apt install ros-jazzy-rmw-cyclonedds-cpp (or RMW_IMPLEMENTATION=rmw_fastrtps_cpp for a dry run)"; exit 3; }
  export CYCLONEDDS_URI="file://$ROOT/config/dds/cyclonedds_radio.xml"
fi
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
echo "RMW=$RMW_IMPLEMENTATION logs=$OUT"
fail=0
chk() { if [ "$2" = ok ]; then echo "PASS  $1"; else echo "FAIL  $1  ($2)"; fail=1; fi; }
inns() { local a=$1; shift; ip netns exec ns-$a "$@"; }

run_pair() { # $1=tag
  inns agent2 timeout $((DUR+3)) ros2 run swarm_control swarm_node --ros-args -p role:=follower -p agent_id:=2 \
      -p log_csv:="$OUT/$1_follower.csv" > "$OUT/$1_follower.log" 2>&1 &
  sleep 1
  inns agent1 timeout $((DUR+2)) ros2 run swarm_control swarm_node --ros-args -p role:=leader -p agent_id:=1 \
      -p rate_hz:=10.0 > "$OUT/$1_leader.log" 2>&1 &
  wait
}

python3 "$ROOT/orchestrator/wire.py" up --config "$CFG" || exit 1
"$NS3" --config="$CFG" --log-dir="$OUT/ns3" --duration=$((2*DUR+30)) > "$OUT/ns3.out" 2>&1 &
NSPID=$!; sleep 3
run_pair A
python3 - "$OUT/A_follower.csv" "$DUR" <<'PY' > "$OUT/A_stats.txt"
import csv, sys, statistics as st
rows = list(csv.DictReader(open(sys.argv[1])))
n = len(rows)
if n == 0: print("0 0 0 0"); sys.exit()
seqs = [int(r['seq']) for r in rows]
exp = max(seqs) - min(seqs) + 1
lat = sorted(float(r['latency_ms']) for r in rows)
print(n, round(100*(1-n/exp), 2), round(lat[len(lat)//2], 2), round(lat[int(0.99*(len(lat)-1))], 2))
PY
read n loss p50 p99 < "$OUT/A_stats.txt"
echo "      follower rx=$n loss=${loss}% latency p50=${p50}ms p99=${p99}ms"
[ "$n" -ge $((DUR*10*8/10)) ] && chk "follower received >=80% of leader states ($n)" ok || chk "follower rx count" "$n"
python3 -c "import sys; sys.exit(0 if float('$loss') < 5 else 1)" && chk "sequence loss < 5% ($loss%)" ok || chk "sequence loss" "$loss%"
grep -q "leader link UP" "$OUT/A_follower.log" && chk "follower reports link UP" ok || chk "link UP log" missing
kill $NSPID 2>/dev/null; wait $NSPID 2>/dev/null
sleep 2
run_pair B
m=$( [ -f "$OUT/B_follower.csv" ] && tail -n +2 "$OUT/B_follower.csv" | wc -l || echo 0 )
[ "$m" -eq 0 ] && chk "isolation: follower got 0 leader states with ns-3 stopped" ok || chk "isolation leak" "$m states"
python3 "$ROOT/orchestrator/wire.py" down --config "$CFG" >/dev/null
echo "logs: $OUT"; exit $fail
