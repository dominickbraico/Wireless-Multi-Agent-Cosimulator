#!/usr/bin/env bash
# Static-link acceptance (local, no Docker): our scenario + config-driven wiring,
# static positions. Checks: ping across radio0, UDP traffic with ~0 loss,
# ns-3 lag within limit, pcap contains 802.11 frames, isolation when ns-3 stops.
#   as root:   sudo tests/static_link.sh
#   rootless:  tests/run_rootless.sh tests/static_link.sh
set -u
# under plain sudo HOME may be /root; use the invoking user's home so tool paths resolve
[ -n "${SUDO_USER:-}" ] && HOME="$(getent passwd "$SUDO_USER" | cut -d: -f6)"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
. "$ROOT/tests/_config_flag.sh"          # sets CFG from --config <yaml> (default: two_agent_wifi.yaml)
NS3="${NS3:-$HOME/dev/ns-3-build/ns-allinone-3.48/ns-3.48/build/scratch/wifi_adhoc_tap/ns3.48-wifi_adhoc_tap-optimized}"
OUT="${OUT:-$(mktemp -d)}"
DUR=${DUR:-32}
[ "${COSIM_SANDBOX:-0}" = 1 ] && mount -t tmpfs tmpfs /run/netns   # private netns dir, rootless mode only
ip link set lo up 2>/dev/null
python3 "$ROOT/orchestrator/wire.py" up --config "$CFG" || exit 1
"$NS3" --config="$CFG" --log-dir="$OUT" --duration=$DUR > "$OUT/ns3.out" 2>&1 &
NSPID=$!
sleep 3
fail=0
chk() { if [ "$2" = ok ]; then echo "PASS  $1"; else echo "FAIL  $1  ($2)"; fail=1; fi; }

p=$(ip netns exec ns-agent1 ping -c 5 -W 2 10.10.0.2 2>&1 | grep -o "[0-9]* received")
[ "$p" = "5 received" ] && chk "ping agent1->agent2 over radio0" ok || chk "ping agent1->agent2" "$p"

# UDP stream agent1 -> agent2 (iperf3 not installed; use a small python sender/receiver)
cat > "$OUT/udp.py" <<'PY'
import socket, sys, time
mode, ip = sys.argv[1], sys.argv[2]
N, SIZE, RATE = 2000, 1000, 2_000_000   # 2 Mbit/s, 1000 B datagrams
if mode == "rx":
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind((ip, 5001)); s.settimeout(6)
    got = set()
    try:
        while True: got.add(int.from_bytes(s.recvfrom(2048)[0][:4], "big"))
    except socket.timeout: pass
    print(len(got), N)
    miss = sorted(set(range(N)) - got); runs = []
    for m in miss:
        if runs and m == runs[-1][1] + 1: runs[-1][1] = m
        else: runs.append([m, m])
    print("missing runs (first 15):", runs[:15], "count", len(runs), file=sys.stderr)
else:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); gap = SIZE*8/RATE; t = time.time()
    for i in range(N):
        s.sendto(i.to_bytes(4, "big") + b"x"*(SIZE-4), (ip, 5001))
        t += gap; time.sleep(max(0, t - time.time()))
PY
ip netns exec ns-agent2 python3 "$OUT/udp.py" rx 10.10.0.2 > "$OUT/udp_rx.txt" 2> "$OUT/udp_rx.err" &
RX=$!; sleep 0.5
ip netns exec ns-agent1 python3 "$OUT/udp.py" tx 10.10.0.2; wait $RX
read got total < "$OUT/udp_rx.txt"
loss=$(python3 -c "print(round(100*(1-$got/$total),2))")
python3 -c "import sys; sys.exit(0 if $loss < 1.0 else 1)" && chk "UDP 2 Mbit/s loss ${loss}% (<1%)" ok || chk "UDP loss" "${loss}%"

# real iperf3 (the brief's tool), UDP 2 Mbit/s for 5 s
if command -v iperf3 >/dev/null; then
  ip netns exec ns-agent2 iperf3 -s -1 -B 10.10.0.2 >/dev/null 2>&1 &
  IPS=$!; sleep 1
  lp=$(ip netns exec ns-agent1 iperf3 -c 10.10.0.2 -u -b 2M -t 5 --json 2>/dev/null | python3 -c "import sys,json; d=json.load(sys.stdin); print(d['end']['sum']['lost_percent'])" 2>/dev/null || echo NA)
  kill $IPS 2>/dev/null
  python3 -c "import sys; sys.exit(0 if float('$lp') < 1.0 else 1)" 2>/dev/null && chk "iperf3 -u 2 Mbit/s loss ${lp}% (<1%)" ok || chk "iperf3 -u loss" "$lp"
fi
wait $NSPID 2>/dev/null
grep -q '"valid_ns3_lag": true' "$OUT/summary.json" && chk "ns-3 lag within limit ($(grep max_lag_ms "$OUT/summary.json" | head -1 | tr -d ' ,'))" ok || chk "ns-3 lag" "$(cat "$OUT/summary.json" 2>/dev/null | tr -d '\n')"
ls "$OUT"/wifi-*.pcap >/dev/null 2>&1 && chk "pcap files written" ok || chk "pcap files" missing
if [ -s "$OUT/wifi-0-0.pcap" ]; then
  link=$(tcpdump -Z root -r "$OUT/wifi-0-0.pcap" -nn 2>/dev/null | head -50 | grep -c "")
  [ "$link" -gt 0 ] && chk "pcap decodes ($link frames sampled)" ok || chk "pcap decode" empty
fi
ip netns exec ns-agent1 ping -c 3 -W 1 10.10.0.2 >/dev/null 2>&1 && chk "isolation: ping after ns-3 exit" "STILL REACHABLE" || chk "isolation: no ping after ns-3 exit" ok
python3 "$ROOT/orchestrator/wire.py" down --config "$CFG" >/dev/null
echo "logs: $OUT"; exit $fail
