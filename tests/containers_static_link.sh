#!/usr/bin/env bash
# Static-link acceptance in Docker Compose: one container per agent plus the ns-3 container, positions static.
# The container counterpart of tests/static_link.sh. Needs only the Docker daemon (no sudo, no host network changes).
#   tests/containers_static_link.sh [--config YAML]      (images first: orchestrator/cosim.py build)
# Checks: ping and 2 Mbit/s UDP agent1 -> agent2 across ns-3 with negligible loss; the agents cannot reach each other
# any other way (management network, link networks); ns-3 lag within limit; pcap holds 802.11 frames; with ns-3 stopped
# the agents cannot reach each other at all.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
. "$ROOT/tests/_config_flag.sh"          # sets CFG from --config <yaml> (default: two_agent_wifi.yaml)
OUT="${OUT:-$(mktemp -d)}"; mkdir -p "$OUT"
cosim() { python3 "$ROOT/orchestrator/cosim.py" "$@"; }
dc() { docker compose -f "$OUT/compose/docker-compose.yml" "$@"; }   # $ROOT may contain spaces
trap 'cosim down --out "$OUT" >/dev/null 2>&1' EXIT
trap 'exit 130' INT TERM HUP
fail=0
chk() { if [ "$2" = ok ]; then echo "PASS  $1"; else echo "FAIL  $1  ($2)"; fail=1; fi; }
plan() { python3 "$ROOT/orchestrator/containers.py" "$CFG" | python3 -c "import json,sys; d=json.load(sys.stdin); print($1)"; }
R2=$(plan "d['agents']['agent2']['radio_ip']"); M2=$(plan "d['agents']['agent2']['mgmt_ip']"); L2=$(plan "d['agents']['agent2']['agent_link_ip']")

echo "logs: $OUT"
cosim up --config "$CFG" --out "$OUT" --mode net --timeout 120 > "$OUT.up.log" 2>&1 || { cat "$OUT.up.log"; chk "stack starts" "see above"; exit 1; }
mv "$OUT.up.log" "$OUT/up.log"
chk "stack starts: ns3 + $(plan "len(d['agents'])") agent containers healthy" ok

p=$(dc exec -T agent1 ping -c 5 -W 2 "$R2" 2>&1 | grep -o "[0-9]* received")
[ "$p" = "5 received" ] && chk "ping agent1->agent2 over radio0" ok || chk "ping agent1->agent2" "$p"

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
else:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); gap = SIZE*8/RATE; t = time.time()
    for i in range(N):
        s.sendto(i.to_bytes(4, "big") + b"x"*(SIZE-4), (ip, 5001))
        t += gap; time.sleep(max(0, t - time.time()))
PY
dc exec -T agent2 python3 /logs/udp.py rx "$R2" > "$OUT/udp_rx.txt" &
RX=$!; sleep 1
dc exec -T agent1 python3 /logs/udp.py tx "$R2"; wait $RX
read got total < "$OUT/udp_rx.txt"
loss=$(python3 -c "print(round(100*(1-$got/$total),2))")
python3 -c "import sys; sys.exit(0 if $loss < 1.0 else 1)" && chk "UDP 2 Mbit/s loss ${loss}% (<1%)" ok || chk "UDP loss" "${loss}%"

# isolation: the only path between agents is the emulated radio
dc exec -T agent1 ping -c 2 -W 1 "$M2" >/dev/null 2>&1 && chk "isolation: agent1 cannot ping agent2 on the management network" "REACHABLE" || chk "isolation: agent1 cannot ping agent2 on the management network" ok
dc exec -T agent1 ping -c 2 -W 1 "$L2" >/dev/null 2>&1 && chk "isolation: agent1 cannot reach agent2's link network" "REACHABLE" || chk "isolation: agent1 cannot reach agent2's link network" ok
peer=$(dc exec -T agent1 nft list counters table inet cosim_mgmt 2>/dev/null | grep -A1 "peer_out" | grep -o "packets [0-9]*" | head -1)
[ -n "$peer" ] && [ "$peer" != "packets 0" ] && chk "management firewall counted the blocked attempt ($peer)" ok || chk "management firewall counter" "${peer:-no counter}"

dc exec -T ns3 sh -c 'ls /logs/ns3/wifi-*.pcap' >/dev/null 2>&1 && chk "pcap files written" ok || chk "pcap files" missing
frames=$(dc exec -T ns3 sh -c 'tcpdump -r /logs/ns3/wifi-0-0.pcap -nn 2>/dev/null | head -50 | wc -l' | tr -d '\r')
[ "${frames:-0}" -gt 0 ] && chk "pcap decodes ($frames frames sampled)" ok || chk "pcap decode" empty

# ns-3 stopped: nothing may get through (the radio is only ever the emulated channel)
dc stop ns3 >/dev/null 2>&1
dc exec -T agent1 ping -c 3 -W 1 "$R2" >/dev/null 2>&1 && chk "isolation: ping after ns-3 stopped" "STILL REACHABLE" || chk "isolation: no ping after ns-3 stopped" ok
grep -q '"valid_ns3_lag": true' "$OUT/ns3/summary.json" 2>/dev/null && chk "ns-3 lag within limit ($(grep max_lag_ms "$OUT/ns3/summary.json" | head -1 | tr -d ' ,'))" ok || chk "ns-3 lag" "$(tr -d '\n' < "$OUT/ns3/summary.json" 2>/dev/null)"
# per-pair signal log (what tools/plot_comm_map.py draws): complete, equal to the YAML link-budget formula for these static
# positions, and equal to the signal every receiver actually measured on the frames it got
res=$(python3 - "$CFG" "$OUT/ns3" "$ROOT/tools" <<'PY'
import csv, sys, collections, math
sys.path.insert(0, sys.argv[3]); import plot_comm_map as pm, yaml
cfg = yaml.safe_load(open(sys.argv[1])); d = sys.argv[2]
names = [a["name"] for a in cfg["agents"]]; n = len(names)
try: rows = list(csv.DictReader(open(d + "/pair_rssi.csv")))
except FileNotFoundError: print("no pair_rssi.csv"); sys.exit()
ticks = collections.defaultdict(dict)
for r in rows: ticks[r["t_s"]][(r["tx"], r["rx"])] = (float(r["rssi_dbm"]), float(r["distance_m"]))
full = [t for t in ticks.values() if len(t) == n * (n - 1)]
if not full: print("no complete sample of %d ordered pairs" % (n * (n - 1))); sys.exit()
worst = max(abs(v - pm.rx_power_dbm(cfg, dist)) for v, dist in full[0].values())
if worst > 0.01: print("logged value differs from the YAML formula by %.3f dB" % worst); sys.exit()
logged = collections.defaultdict(set)
for (tx, rx), (v, _) in full[0].items(): logged[rx].add(v)
meas = collections.defaultdict(list)
for r in csv.DictReader(open(d + "/rssi.csv")): meas[names[int(r["rx_node"])]].append(float(r["signal_dbm"]))
bad = [(rx, s) for rx, sig in meas.items() for s in set(round(x, 2) for x in sig) if not any(abs(s - v) < 0.01 for v in logged[rx])]
print("ok" if not bad and meas else "measured signal not in the pair log: %s" % bad[:3])
PY
)
[ "$res" = ok ] && chk "ns-3 pair signal log: complete, equals the YAML formula and the signal the receivers measured" ok || chk "ns-3 pair signal log" "$res"
want=$(python3 -c "import yaml,sys; w=yaml.safe_load(open(sys.argv[1]))['network']['wifi']; print(w['standard'], w['data_mode'])" "$CFG")
got=$(python3 -c "import json,sys; d=json.load(open(sys.argv[1])); print(d['wifi_standard'], d['wifi_data_mode'])" "$OUT/ns3/summary.json" 2>/dev/null)
[ "$want" = "$got" ] && chk "ns-3 emulated the configured technology (standard / data rate: $got)" ok || chk "configured technology reached ns-3" "wanted '$want', ns-3 ran '${got:-nothing}'"
echo "logs: $OUT"; exit $fail
