#!/usr/bin/env bash
# Isaac Sim spike, stage D: the pose feed from Isaac Sim to ns-3.
#   COSIM_ACCEPT_ISAAC_EULA=Y tests/isaac_pose_feed.sh [--config YAML] [--timeout S]     (images first: python3 orchestrator/cosim.py build --config YAML)
# Part A (no Docker): adapters/isaac_sim/pose_feed.py against the pose-feed contract (docs/CONTAINERS.md):
#   A1. every datagram is JSON with exactly the keys seq, t_sim, t_wall, agents; positions are the stored position plus antenna_offset
#   A2. at the configured rate (within +-30 %), seq rising by 1 each time, t_wall non-decreasing
#   A3. when the simulator stalls (no update for 1.5 s) the last pose keeps being sent at the same rate, and t_wall keeps advancing
#   A4. a new position is sent after the next update
# Part B (full stack, needs the NVIDIA runtime and the Isaac image): agent1's PX4 is armed and told to take off; in ns3/pose_applied.csv (what ns-3 applied)
#   B1. the antenna z rises, from before arming to its maximum, by MIS_TAKEOFF_ALT +- 0.5 m (the commanded altitude; read from PX4)
#   B2. pose samples for agent1 reach ns-3 throughout (no gap longer than 1.0 s between consecutive applied samples while flying)
# Criteria fixed before the first run (docs/EXPERIMENTS.md, "Isaac Sim stage D").
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CFG="$ROOT/config/experiments/isaac_one_agent.yaml"; TIMEOUT=600
while [ $# -gt 0 ]; do
  case "$1" in
    --config) CFG="$2"; shift 2;;
    --timeout) TIMEOUT="$2"; shift 2;;
    *) echo "usage: $0 [--config YAML] [--timeout S]"; exit 2;;
  esac
done
OUT="${OUT:-$(mktemp -d)}"; mkdir -p "$OUT"
cosim() { python3 "$ROOT/orchestrator/cosim.py" "$@"; }
trap 'cosim down --out "$OUT" >/dev/null 2>&1' EXIT
trap 'exit 130' INT TERM HUP
fail=0
chk() { if [ "$2" = ok ]; then echo "PASS  $1"; else echo "FAIL  $1  ($2)"; fail=1; fi; }

# ---- Part A
a=$(ROOT="$ROOT" python3 - <<'E' 2>&1
import json, os, socket, sys, time
sys.path.insert(0, os.path.join(os.environ["ROOT"], "adapters", "isaac_sim"))
import pose_feed

srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
srv.bind(("127.0.0.1", 0)); srv.settimeout(0.2)
RATE = 20.0
s = pose_feed.PoseSender(srv.getsockname(), RATE, {"agent1": [0.0, 0.0, 0.1]})
s.update("agent1", [1.0, 2.0, 3.0], 5.0)
s.start()

def collect(seconds):
    got, end = [], time.time() + seconds
    while time.time() < end:
        try:
            got.append(json.loads(srv.recv(65536)))
        except socket.timeout:
            pass
    return got

g1 = collect(1.0)
ok1 = bool(g1) and all(set(m) == {"seq", "t_sim", "t_wall", "agents"} and m["agents"] == {"agent1": [1.0, 2.0, 3.1]} for m in g1)
print("A1", "ok" if ok1 else f"bad datagram: {g1[:1]}")
seqs = [m["seq"] for m in g1]; tw = [m["t_wall"] for m in g1]
rate = len(g1) / 1.0
okr = abs(rate - RATE) <= 0.3 * RATE and all(b - a == 1 for a, b in zip(seqs, seqs[1:])) and all(b >= a for a, b in zip(tw, tw[1:]))
print("A2", "ok" if okr else f"rate {rate:.1f}/s, seqs {seqs[:5]}")
time.sleep(1.5)                                     # simulator stalled: no update() calls; the receiver is not reading, the kernel buffers
srv.settimeout(0.05)
while True:                                         # drain what accumulated while stalled
    try: srv.recv(65536)
    except socket.timeout: break
srv.settimeout(0.2)
g2 = collect(1.0)                                   # still no update(): must still arrive at the same rate with the same pose
same = all(m["agents"] == {"agent1": [1.0, 2.0, 3.1]} for m in g2)
adv = bool(g2) and g2[-1]["t_wall"] > g1[-1]["t_wall"] + 1.4
okstall = len(g2) >= 0.7 * RATE and same and adv and all(m["t_sim"] == 5.0 for m in g2)
print("A3", "ok" if okstall else f"{len(g2)} datagrams, same pose {same}, t_wall advancing {adv}")
s.update("agent1", [4.0, 5.0, 6.0], 6.0)
g3 = collect(0.5)
ok4 = bool(g3) and g3[-1]["agents"] == {"agent1": [4.0, 5.0, 6.1]} and g3[-1]["t_sim"] == 6.0
print("A4", "ok" if ok4 else f"last {g3[-1:] }")
s.stop()
E
)
for k in A1 A2 A3 A4; do
  line=$(echo "$a" | grep "^$k " | head -1)
  name=$(case $k in A1) echo "contract: keys and antenna offset";; A2) echo "rate, seq and t_wall";; A3) echo "stalled simulator: last pose keeps being sent";; A4) echo "new position is sent after an update";; esac)
  case "$line" in "$k ok") chk "$name" ok;; *) chk "$name" "${line:-$(echo "$a" | tail -1)}";; esac
done
[ $fail = 0 ] || { echo "RESULT: FAIL (part A)"; exit 1; }

# ---- Part B
if [ "${COSIM_ACCEPT_ISAAC_EULA:-}" != Y ]; then
  chk "part B needs COSIM_ACCEPT_ISAAC_EULA=Y (NVIDIA Omniverse licence agreement, accepted by you)" "variable not set"
  echo "RESULT: FAIL"; exit 1
fi
echo "logs: $OUT"
cosim up --config "$CFG" --out "$OUT" --timeout "$TIMEOUT" > "$OUT.up.log" 2>&1; rc=$?
mv "$OUT.up.log" "$OUT/up.log"
[ $rc = 0 ] && [ -f "$OUT/agent1.px4_ready" ] || { chk "cosim up: stack healthy and agent1 reports PX4 ready" "exit $rc; see $OUT/up.log"; echo "RESULT: FAIL"; exit 1; }
chk "cosim up: stack healthy and agent1 reports PX4 ready" ok

dc() { docker compose -f "$OUT/compose/docker-compose.yml" "$@"; }
BIN=/opt/PX4-Autopilot/build/px4_sitl_default; WD=$BIN/instance_0
px() { dc exec -T -w "$WD" -e LD_LIBRARY_PATH=/usr/local/lib agent1 "$BIN/bin/px4-$1" --instance 0 "${@:2}" 2>&1; }
ALT=$(px param show -q MIS_TAKEOFF_ALT | grep -oE '^[0-9]+(\.[0-9]+)?$' | tail -1)     # -q prints the bare value; plain `show` prints nothing for a parameter not yet used
[ -n "$ALT" ] || { chk "read MIS_TAKEOFF_ALT from PX4" "no value"; echo "RESULT: FAIL"; exit 1; }
sleep 10                                             # let the vehicle settle on the ground and the pose feed run
T_ARM=$(python3 -c 'import time; print(time.time())')
px commander arm >/dev/null; px commander takeoff >/dev/null
sleep 45
dc stop -t 25 ns3 >/dev/null 2>&1                    # ns-3 flushes pose_applied.csv on shutdown

python3 - "$OUT/ns3/pose_applied.csv" "$T_ARM" "$ALT" <<'E' > "$OUT/pose_check.txt" 2>&1
import csv, sys
rows = [r for r in csv.DictReader(open(sys.argv[1])) if int(r["node"]) == 0]
t_arm, alt = float(sys.argv[2]), float(sys.argv[3])
before = [float(r["z"]) for r in rows if float(r["t_wall_apply_s"]) < t_arm]
after = [r for r in rows if float(r["t_wall_apply_s"]) >= t_arm]
if not before or not after:
    print(f"B1 no samples (before {len(before)}, after {len(after)})"); sys.exit(0)
z0 = sum(before[-20:]) / len(before[-20:])
rise = max(float(r["z"]) for r in after) - z0
print(("B1 ok" if abs(rise - alt) <= 0.5 else "B1 bad") + f" rise {rise:.2f} m, MIS_TAKEOFF_ALT {alt:.2f} m, z before {z0:.2f} m")
t = [float(r["t_wall_apply_s"]) for r in after]
gap = max(b - a for a, b in zip(t, t[1:])) if len(t) > 1 else 99
print(("B2 ok" if gap <= 1.0 else "B2 bad") + f" longest gap {gap:.2f} s over {len(t)} samples")
E
cat "$OUT/pose_check.txt" | sed 's/^/  /'
grep -q "^B1 ok" "$OUT/pose_check.txt" && chk "ns-3 applied z rose by the commanded takeoff altitude +-0.5 m" ok || chk "ns-3 applied z rose by the commanded takeoff altitude +-0.5 m" "$(grep '^B1' "$OUT/pose_check.txt")"
grep -q "^B2 ok" "$OUT/pose_check.txt" && chk "pose samples reached ns-3 without a gap over 1.0 s" ok || chk "pose samples reached ns-3 without a gap over 1.0 s" "$(grep '^B2' "$OUT/pose_check.txt")"

echo "RESULT: $([ $fail = 0 ] && echo PASS || echo FAIL)"
exit $fail
