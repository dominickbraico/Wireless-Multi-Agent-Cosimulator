#!/usr/bin/env bash
# ROS 2 isolation acceptance (local): ROS 2 talker/listener across the emulated link,
# plus the isolation guard: with ns-3 stopped there must be ZERO messages and
# zero discovery.
#   sudo tests/ros2_isolation.sh      or      tests/run_rootless.sh tests/ros2_isolation.sh
# RMW defaults to rmw_cyclonedds_cpp (the supported RMW).
set -u
# under plain sudo HOME may be /root; use the invoking user's home so tool paths resolve
[ -n "${SUDO_USER:-}" ] && HOME="$(getent passwd "$SUDO_USER" | cut -d: -f6)"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
. "$ROOT/tests/_config_flag.sh"          # sets CFG from --config <yaml> (default: two_agent_wifi.yaml)
NS3="${NS3:-$HOME/dev/ns-3-build/ns-allinone-3.48/ns-3.48/build/scratch/wifi_adhoc_tap/ns3.48-wifi_adhoc_tap-optimized}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
OUT="${OUT:-$(mktemp -d)}"
[ "${COSIM_SANDBOX:-0}" = 1 ] && mount -t tmpfs tmpfs /run/netns
ip link set lo up 2>/dev/null
set +u; source /opt/ros/jazzy/setup.bash; set -u
if [ "$RMW_IMPLEMENTATION" = rmw_cyclonedds_cpp ] && [ ! -d /opt/ros/jazzy/share/rmw_cyclonedds_cpp ]; then
  echo "rmw_cyclonedds_cpp missing: sudo apt install ros-jazzy-rmw-cyclonedds-cpp"; exit 3
fi
[ "$RMW_IMPLEMENTATION" = rmw_cyclonedds_cpp ] && export CYCLONEDDS_URI="file://$ROOT/config/dds/cyclonedds_radio.xml"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
echo "RMW=$RMW_IMPLEMENTATION domain=$ROS_DOMAIN_ID logs=$OUT"
fail=0
chk() { if [ "$2" = ok ]; then echo "PASS  $1"; else echo "FAIL  $1  ($2)"; fail=1; fi; }
inns() { local a=$1; shift; ip netns exec ns-$a "$@"; }

run_pair() { # $1=tag $2=seconds
  inns agent2 timeout "$2" ros2 run demo_nodes_cpp listener > "$OUT/$1_listener.txt" 2>&1 &
  sleep 1
  inns agent1 timeout "$2" ros2 run demo_nodes_cpp talker > "$OUT/$1_talker.txt" 2>&1 &
  wait
}

python3 "$ROOT/orchestrator/wire.py" up --config "$CFG" || exit 1

# --- A: with ns-3 up
"$NS3" --config="$CFG" --log-dir="$OUT/ns3" --duration=40 > "$OUT/ns3.out" 2>&1 &
NSPID=$!
sleep 3
run_pair A 14
n=$(grep -c "I heard" "$OUT/A_listener.txt")
[ "$n" -ge 5 ] && chk "talker->listener through ns-3 ($n msgs in 14 s)" ok || chk "talker->listener through ns-3" "$n msgs"
# traffic really crossed the emulated channel as RTPS over 802.11
sleep 1
kill $NSPID 2>/dev/null; wait $NSPID 2>/dev/null
rtps=$(tcpdump -Z root -r "$OUT/ns3/wifi-1-0.pcap" -nn 'udp and (portrange 7400-7500)' 2>/dev/null | wc -l)
[ "$rtps" -gt 0 ] && chk "RTPS (UDP 74xx) frames in ns-3 pcap ($rtps)" ok || chk "RTPS in pcap" "$rtps frames"

# --- B: ns-3 stopped -> nothing may get through (incl. discovery)
sleep 2
run_pair B 14
m=$(grep -c "I heard" "$OUT/B_listener.txt")
[ "$m" -eq 0 ] && chk "isolation: 0 messages with ns-3 stopped" ok || chk "isolation: messages leaked" "$m msgs"
inns agent1 timeout 14 ros2 run demo_nodes_cpp talker >/dev/null 2>&1 &
sleep 3
d=$(inns agent2 timeout 8 ros2 topic list --no-daemon 2>/dev/null | grep -c "^/chatter$")
wait
[ "$d" -eq 0 ] && chk "isolation: agent2 does not discover agent1's topic" ok || chk "isolation: discovery leaked" "$d"
python3 "$ROOT/orchestrator/wire.py" down --config "$CFG" >/dev/null
echo "logs: $OUT"; exit $fail
