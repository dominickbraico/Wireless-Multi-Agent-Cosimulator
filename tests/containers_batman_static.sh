#!/usr/bin/env bash
# batman-adv mesh routing on a static line of three agents (network only: no simulator, no PX4), in Docker Compose.
# agent1 and agent3 are out of range of each other; agent2 sits between them. Criteria (docs/EXPERIMENTS.md, "batman-adv mesh"):
#   S1 without batman-adv: agent1 -> agent3 answers 0 of 20 pings
#   S2 with batman-adv:    agent1 -> agent3 answers at least 19 of 20 pings
#   S3 `batctl tr` from agent1 to agent3 lists agent2 and then agent3
#   S4 a capture on agent2 holds the ping frames arriving from agent1 and leaving to agent3 with a TTL one lower (relayed)
#   S5 every frame the agents send toward ns-3 is a batman-adv frame; the mesh0 IP firewall counters are 0
#   S6 10 pings of 3000 bytes agent1 -> agent3, at least 9 answered
#   S7 within 30 s of batman-adv starting every ordered pair answers at least 4 of 5 pings (the time taken is reported)
#   tests/containers_batman_static.sh [--config YAML]     (default batman_line_static.yaml; the host needs the batman_adv module)
# Images first: python3 orchestrator/cosim.py build. Needs only the Docker daemon and the kernel module.
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CFG="$ROOT/config/experiments/batman_line_static.yaml"
. "$ROOT/tests/_config_flag.sh"
OUT="${OUT:-$ROOT/logs/batman_static_$(date +%Y%m%d_%H%M%S)}"; mkdir -p "$OUT"
cosim() { python3 "$ROOT/orchestrator/cosim.py" "$@"; }
FAIL=0
check() { if [ "$2" = ok ]; then echo "PASS  $1"; else echo "FAIL  $1  ($3)"; FAIL=1; fi; }
received() { grep -o '[0-9]* received' | grep -o '^[0-9]*' || echo 0; }

# the three agents, their radio IPs and radio MACs (= batman-adv originator addresses), from the YAML and containers.py
read -r -a NAMES <<< "$(python3 -c "import yaml,sys; print(' '.join(a['name'] for a in yaml.safe_load(open(sys.argv[1]))['agents']))" "$CFG")"
read -r -a IPS <<< "$(python3 -c "import yaml,sys; print(' '.join(a['radio_ip'] for a in yaml.safe_load(open(sys.argv[1]))['agents']))" "$CFG")"
read -r -a MACS <<< "$(cd "$ROOT/orchestrator" && python3 -c "import containers,yaml,sys; c=yaml.safe_load(open(sys.argv[1])); p=containers.plan(c)['agents']; print(' '.join(p[a['name']]['mac'] for a in c['agents']))" "$CFG")"
[ ${#NAMES[@]} = 3 ] || { echo "FAIL  the configuration must have exactly three agents in a line"; exit 2; }
A1=${NAMES[0]} A2=${NAMES[1]} A3=${NAMES[2]}

# ---- S1: the same layout without batman-adv
python3 - "$CFG" "$OUT/no_mesh.yaml" <<'PY' || exit 2
import sys, yaml
c = yaml.safe_load(open(sys.argv[1])); c["network"].pop("mesh", None); c["experiment"]["name"] += "_no_mesh"
yaml.safe_dump(c, open(sys.argv[2], "w"), sort_keys=False)
PY
mkdir -p "$OUT/no_mesh"
trap 'cosim down --out "$OUT/no_mesh" >/dev/null 2>&1; cosim down --out "$OUT/mesh" >/dev/null 2>&1' EXIT
trap 'exit 130' INT TERM HUP
cosim up --config "$OUT/no_mesh.yaml" --out "$OUT/no_mesh" --mode net --timeout 180 > "$OUT/no_mesh/up.log" 2>&1 || { tail -20 "$OUT/no_mesh/up.log"; echo "FAIL  stack without mesh not healthy"; exit 1; }
dcn() { docker compose -f "$OUT/no_mesh/compose/docker-compose.yml" "$@"; }
n=$(dcn exec -T "$A1" ping -c 20 -i 0.2 -W 1 "${IPS[2]}" 2>&1 | received)
nb=$(dcn exec -T "$A1" ping -c 5 -i 0.2 -W 1 "${IPS[1]}" 2>&1 | received)
[ "$n" = 0 ] && [ "$nb" -ge 4 ] && check "S1 without batman-adv: $A1 -> $A3 0 of 20 (neighbour $A2 answers $nb of 5)" ok || check "S1 without batman-adv" bad "$A1 -> $A3 $n of 20, -> $A2 $nb of 5"
cosim down --out "$OUT/no_mesh" >/dev/null 2>&1

# ---- with batman-adv
mkdir -p "$OUT/mesh"
cosim up --config "$CFG" --out "$OUT/mesh" --mode net --timeout 180 > "$OUT/mesh/up.log" 2>&1 || { tail -20 "$OUT/mesh/up.log"; echo "FAIL  stack with mesh not healthy"; exit 1; }
dc() { docker compose -f "$OUT/mesh/compose/docker-compose.yml" "$@"; }
t0=$(date +%s.%N)

# S7: poll every ordered pair until each answers, at most 30 s from batman-adv start (the stack reports healthy once radio0 has its IP)
declare -A OKPAIR; tconv=""
while :; do
  left=0
  for i in 0 1 2; do for j in 0 1 2; do [ $i = $j ] && continue; k="$i$j"; [ -n "${OKPAIR[$k]:-}" ] && continue
    if dc exec -T "${NAMES[$i]}" ping -c 1 -W 1 "${IPS[$j]}" >/dev/null 2>&1; then
      OKPAIR[$k]=1; echo "${NAMES[$i]} -> ${NAMES[$j]} first answer $(python3 -c "import sys; print(f'{float(sys.argv[1]) - float(sys.argv[2]):.1f}')" "$(date +%s.%N)" "$t0") s" >> "$OUT/mesh/s7_first_answer.txt"
    else left=1; fi
  done; done
  el=$(python3 -c "import sys; print(f'{float(sys.argv[1]) - float(sys.argv[2]):.1f}')" "$(date +%s.%N)" "$t0")
  [ $left = 0 ] && { tconv=$el; break; }
  python3 -c "import sys; sys.exit(0 if float(sys.argv[1]) > 30 else 1)" "$el" && break
done
pairs_ok=1; detail=""
for i in 0 1 2; do for j in 0 1 2; do [ $i = $j ] && continue
  n=$(dc exec -T "${NAMES[$i]}" ping -c 5 -i 0.2 -W 1 "${IPS[$j]}" 2>&1 | received); detail="$detail ${NAMES[$i]}->${NAMES[$j]} $n/5"; [ "$n" -ge 4 ] || pairs_ok=0
done; done
[ -n "$tconv" ] && [ $pairs_ok = 1 ] && check "S7 every ordered pair reachable ${tconv} s after start;$detail" ok || check "S7 every ordered pair reachable within 30 s" bad "first reply from all pairs: ${tconv:-not within 30 s};$detail"
for a in "${NAMES[@]}"; do { echo "== $a originators"; dc exec -T "$a" batctl meshif radio0 o; echo "== $a neighbours"; dc exec -T "$a" batctl meshif radio0 n; } >> "$OUT/mesh/batctl_tables.txt" 2>&1; done

# S2 and S4: 20 pings agent1 -> agent3 while agent2 captures its mesh0
dc exec -T "$A2" sh -c "timeout 10 tcpdump -i mesh0 -U -w /logs/relay_${A2}_mesh0.pcap" >/dev/null 2>&1 & CAP=$!
for a in "$A1" "$A3"; do dc exec -T "$a" sh -c "timeout 10 tcpdump -i mesh0 -U -w /logs/${a}_mesh0.pcap" >/dev/null 2>&1 & done
sleep 1.5
out=$(dc exec -T "$A1" ping -c 20 -i 0.2 -W 2 "${IPS[2]}" 2>&1); n=$(echo "$out" | received)
echo "$out" > "$OUT/mesh/s2_ping.txt"
[ "$n" -ge 19 ] && check "S2 with batman-adv: $A1 -> $A3 $n of 20 ($(echo "$out" | grep -o 'rtt.*'))" ok || check "S2 with batman-adv" bad "$n of 20"
wait $CAP 2>/dev/null; sleep 1

# S3: batman-adv traceroute to agent3's originator address
tr=$(dc exec -T "$A1" batctl meshif radio0 tr "${MACS[2]}" 2>&1); echo "$tr" > "$OUT/mesh/s3_traceroute.txt"
hops=$(echo "$tr" | grep -E '^ *[0-9]+: ' | awk '{print $2}' | tr '\n' ' ')
[ "$hops" = "${MACS[1]} ${MACS[2]} " ] && check "S3 batctl tr $A1 -> $A3: two hops, $A2 (${MACS[1]}) then $A3" ok || check "S3 batctl tr two hops via $A2" bad "hops: '$hops', $OUT/mesh/s3_traceroute.txt"

# S4: the relay in agent2's capture
python3 "$ROOT/tools/batman_pcap.py" "$OUT/mesh/relay_${A2}_mesh0.pcap" > "$OUT/mesh/relay_${A2}.txt"
rel=$(python3 - "$ROOT/tools" "$OUT/mesh/relay_${A2}_mesh0.pcap" "${MACS[0]}" "${MACS[1]}" "${MACS[2]}" "${IPS[0]}" "${IPS[2]}" <<'PY'
import sys
sys.path.insert(0, sys.argv[1]); import batman_pcap
_, flows = batman_pcap.decode(sys.argv[2]); m1, m2, m3, ip1, ip3 = sys.argv[3:8]
def n(src, dst, s, d):
    return {ttl: c for (a, b, x, y, ttl), c in flows.items() if (a, b, x, y) == (src, dst, s, d)}
inn, outn = n(m1, m2, ip1, ip3), n(m2, m3, ip1, ip3)
back_in, back_out = n(m3, m2, ip3, ip1), n(m2, m1, ip3, ip1)
ok = all(sum(v.values()) >= 19 for v in (inn, outn, back_in, back_out)) and all(t - 1 in outn for t in inn) and all(t - 1 in back_out for t in back_in)
print(("ok" if ok else "bad") + f" in {inn} out {outn} back-in {back_in} back-out {back_out}")
PY
)
[ "${rel%% *}" = ok ] && check "S4 $A2 relays: requests in from $A1 and out to $A3, replies back, TTL lower by one (${rel#ok })" ok || check "S4 $A2 relays" bad "$rel"

# S5: only batman-adv frames toward ns-3, and no IP on mesh0
nonbat=0; for a in "${NAMES[@]}"; do
  f="$OUT/mesh/${a}_mesh0.pcap"; [ "$a" = "$A2" ] && f="$OUT/mesh/relay_${A2}_mesh0.pcap"
  c=$(python3 "$ROOT/tools/batman_pcap.py" --json --from "${MACS[$(printf '%s\n' "${NAMES[@]}" | grep -nx "$a" | cut -d: -f1) - 1]}" "$f" | python3 -c "import json,sys; k=json.load(sys.stdin)['kinds']; print(sum(v for n, v in k.items() if n.startswith('non-batman')))")
  nonbat=$((nonbat + c))
done
ipm=$(for a in "${NAMES[@]}"; do dc exec -T "$a" nft list counters table inet cosim_mesh 2>/dev/null | grep -o 'packets [0-9]*' | grep -o '[0-9]*'; done | paste -sd+ | bc 2>/dev/null)
[ "$nonbat" = 0 ] && [ "${ipm:-x}" = 0 ] && check "S5 only batman-adv frames on mesh0 (three captures), mesh0 IP firewall counters 0" ok || check "S5 only batman-adv frames toward ns-3" bad "non-batman frames $nonbat, mesh0 IP drops ${ipm:-no counters}"

# S6: large packets across two hops
n=$(dc exec -T "$A1" ping -c 10 -i 0.3 -W 2 -s 3000 "${IPS[2]}" 2>&1 | received)
[ "$n" -ge 9 ] && check "S6 3000-byte pings $A1 -> $A3: $n of 10" ok || check "S6 3000-byte pings" bad "$n of 10"

dc logs --no-color > "$OUT/mesh/containers.log" 2>&1
echo "logs: $OUT"; exit $FAIL
