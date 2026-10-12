#!/usr/bin/env bash
# Stop everything the demo/tests start and remove their network plumbing. Safe to run at any time. Needs sudo.
#   demo/cleanup.sh [--config YAML]
#   --config Y  tear down the plumbing of this experiment only (default: every YAML in config/experiments/)
# (The [x]yz pattern trick stops pkill from matching its own command line.)
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CFGS=()
while [ $# -gt 0 ]; do case "$1" in
  --config) CFGS+=("${2:?--config needs a path}"); shift 2;;
  --config=*) CFGS+=("${1#--config=}"); shift;;
  *) echo "unknown option: $1 (supported: --config <yaml>)"; exit 2;; esac; done
[ ${#CFGS[@]} -gt 0 ] || CFGS=("$ROOT"/config/experiments/*.yaml)
sudo -v || exit 1
for pat in "[s]warm_node" "[f]ollower_flight" "[s]quare_trajectory" "[w]ifi_adhoc_tap" "[p]ose_bridge.py" "[r]un_stack.py" \
           "[M]icroXRCEAgent" "bin/[p]x4" "[g]z sim"; do
  sudo pkill -KILL -f "$pat" 2>/dev/null
done
sleep 1
sudo rm -f /tmp/px4-sock-* /tmp/px4_lock-* 2>/dev/null   # stale PX4 daemon sockets/locks from killed runs
for cfg in "${CFGS[@]}"; do
  sudo python3 "$ROOT/orchestrator/wire.py" down --config "$cfg" >/dev/null 2>&1
done
sudo "$ROOT/orchestrator/netns_link.sh" down >/dev/null 2>&1
echo "still running (should be empty):"
pgrep -af "[p]x4 -i|[M]icroXRCEAgent|[g]z sim|[w]ifi_adhoc_tap|[s]warm_node" || echo "  nothing"
echo "network namespaces left (should be none of ns-agent*):"
ip netns list 2>/dev/null | grep -E "ns-agent|ns-left|ns-right" || echo "  none"
ip -br link 2>/dev/null | grep -E "br-mgmt|br-a[0-9]|tap-a[0-9]" || echo "  no bridges/taps"
