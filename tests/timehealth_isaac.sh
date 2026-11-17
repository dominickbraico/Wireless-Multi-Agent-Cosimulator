#!/usr/bin/env bash
# Isaac Sim spike, stage E: the real-time-factor source for Isaac Sim runs (tools/timehealth.py rtf_from_poses, tools/analyze_isaac.py).
#   tests/timehealth_isaac.sh [--logs DIR --config YAML]       DIR: log directory of a run (for example the OUT of tests/isaac_pose_feed.sh)
# Isaac Sim has no equivalent of Gazebo's /stats topic (bridge/rtf.csv), so the factor is taken from the pose feed: simulated seconds (t_sim) over wall
# seconds (t_wall at the moment ns-3 applied the pose) across 5 consecutive applied samples (about 0.1 s at 50 Hz), per agent.
# Part A, synthetic pose streams (no Docker):
#   A1. a stream that advances 1 simulated second per wall second gives RTF 1.0 +-0.02 everywhere
#   A2. a stream at half speed gives 0.5 +-0.02
#   A3. a 2-s stall (t_sim constant, t_wall advancing, as the pose feed repeats the last pose) gives RTF 0 inside the stall, and the 5-s sliding mean over it is < 0.95
#   A4. fewer than 6 samples give no RTF samples (no number is invented)
#   A5. a synthetic run that flies at 1.5x wall time still meets the lower-bound criterion (no threshold changed) but the analyzer prints a "WARN  not paced to wall time"
#       line, and prints the highest 5-s window mean (1.5 +-0.05) and the flight-window mean, so every RTF figure in the docs has a command that prints it
#   A6. a synthetic run at real time (1.0) prints no such warning
# Part B, a real run directory (only with --logs): tools/analyze_isaac.py prints either "RTF ..." with a measured number or "run not certified: <reason>",
#   never neither; with a directory that has no pose samples it prints "run not certified". Whether the run meets time.min_rtf is reported, not asserted here.
# Criteria fixed before the first run (docs/EXPERIMENTS.md, "Isaac Sim stage E").
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LOGS=""; CFG="$ROOT/config/experiments/isaac_one_agent.yaml"
while [ $# -gt 0 ]; do
  case "$1" in
    --logs) LOGS="$2"; shift 2;;
    --config) CFG="$2"; shift 2;;
    *) echo "usage: $0 [--logs DIR] [--config YAML]"; exit 2;;
  esac
done
fail=0
chk() { if [ "$2" = ok ]; then echo "PASS  $1"; else echo "FAIL  $1  ($2)"; fail=1; fi; }

a=$(ROOT="$ROOT" python3 - <<'E' 2>&1
import os, sys
sys.path.insert(0, os.path.join(os.environ["ROOT"], "tools"))
import timehealth

def rows(n, dt_wall, dt_sim, stall=None, t0=1000.0):
    out, ts, tw = [], 0.0, t0
    for i in range(n):
        out.append({"node": "0", "t_sim_s": f"{ts:.6f}", "t_wall_apply_s": f"{tw:.6f}"})
        tw += dt_wall
        if not (stall and stall[0] <= i * dt_wall < stall[1]):
            ts += dt_sim
    return out

def rtf(r):
    return timehealth.rtf_from_poses(r, node=0)

s1 = rtf(rows(500, 0.02, 0.02))                                   # 10 s at 50 Hz, real time
print("A1", "ok" if s1 and all(abs(v - 1.0) <= 0.02 for _, v in s1) else f"{s1[:2]} n={len(s1)}")
s2 = rtf(rows(500, 0.02, 0.01))
print("A2", "ok" if s2 and all(abs(v - 0.5) <= 0.02 for _, v in s2) else f"{s2[:2]} n={len(s2)}")
r3 = rows(500, 0.02, 0.02, stall=(4.0, 6.0))
s3 = rtf(r3)
inside = [v for t, v in s3 if 4.2 <= t - 1000.0 <= 5.9]
chk = timehealth.rtf_check(s3, s3[0][0], s3[-1][0], 0.95, 5.0)
print("A3", "ok" if inside and max(inside) <= 0.02 and chk["worst_window_mean"] < 0.95 and not chk["ok"] else f"inside max {max(inside) if inside else None}, worst window {chk['worst_window_mean']}")
print("A4", "ok" if rtf(rows(5, 0.02, 0.02)) == [] else "samples from 5 rows")
E
)
for k in A1 A2 A3 A4; do
  line=$(echo "$a" | grep "^$k " | head -1)
  name=$(case $k in A1) echo "real-time stream gives RTF 1.0";; A2) echo "half-speed stream gives RTF 0.5";; A3) echo "a stall shows as RTF 0 and fails the sliding mean";; A4) echo "too few samples give no RTF";; esac)
  case "$line" in "$k ok") chk "$name" ok;; *) chk "$name" "${line:-$(echo "$a" | tail -1)}";; esac
done

synth() {      # $1 = simulated seconds per wall second; a 40-s stream at 50 Hz, resting for 12 s then rising 3 m; prints the directory
  d=$(mktemp -d); mkdir -p "$d/ns3"
  python3 - "$d/ns3/pose_applied.csv" "$1" <<'PYE'
import sys
f = open(sys.argv[1], "w"); k = float(sys.argv[2])
f.write("t_ns3_s,node,seq,t_sim_s,x,y,z,stale_ms,t_wall_apply_s\n")
for i in range(2000):
    t = i * 0.02
    z = 0.1 if t < 12 else 0.1 + min(3.0, (t - 12) * 0.5)
    f.write(f"{t:.3f},0,{i},{t * k:.6f},0,0,{z:.3f},0,{1000.0 + t:.6f}\n")
PYE
  echo '{"valid_ns3_lag": true, "ns3_max_lag_ms": 0.1}' > "$d/ns3/summary.json"; echo "$d"
}
d15=$(synth 1.5); o15=$(python3 "$ROOT/tools/analyze_isaac.py" "$d15" "$CFG" 2>&1)
echo "$o15" | grep -q "^PASS  RTF flight window" && echo "$o15" | grep -q "^WARN  not paced to wall time" && echo "$o15" | grep -qE "highest 5-s window mean 1\.(4[5-9]|5[0-5])" && echo "$o15" | grep -qE "flight window \(information\): mean 1\.(4[5-9]|5[0-5])" && chk "A5 1.5x run: criterion met, WARN printed, highest window mean and flight mean printed" ok || chk "A5 1.5x run: criterion met, WARN printed, highest window mean and flight mean printed" "$(echo "$o15" | tr '\n' '|' | cut -c1-300)"
d10=$(synth 1.0); o10=$(python3 "$ROOT/tools/analyze_isaac.py" "$d10" "$CFG" 2>&1)
echo "$o10" | grep -q "^PASS  RTF flight window" && ! echo "$o10" | grep -q "^WARN" && chk "A6 real-time run: criterion met, no WARN" ok || chk "A6 real-time run: criterion met, no WARN" "$(echo "$o10" | tr '\n' '|' | cut -c1-300)"
rm -r "$d15" "$d10"

if [ -n "$LOGS" ]; then
  out=$(python3 "$ROOT/tools/analyze_isaac.py" "$LOGS" "$CFG" 2>&1); rc=$?
  echo "$out" | sed 's/^/  | /'
  if echo "$out" | grep -qE "^(PASS|FAIL)? *RTF .*[0-9]\.[0-9]" || echo "$out" | grep -q "run not certified"; then chk "B1 analyze_isaac.py prints a measured RTF or 'run not certified'" ok; else chk "B1 analyze_isaac.py prints a measured RTF or 'run not certified'" "neither found (exit $rc)"; fi
  empty=$(mktemp -d); mkdir -p "$empty/ns3"; echo "t_ns3_s,node,seq,t_sim_s,x,y,z,stale_ms,t_wall_apply_s" > "$empty/ns3/pose_applied.csv"
  out2=$(python3 "$ROOT/tools/analyze_isaac.py" "$empty" "$CFG" 2>&1)
  echo "$out2" | grep -q "run not certified" && chk "B2 a run without pose samples is 'run not certified'" ok || chk "B2 a run without pose samples is 'run not certified'" "$(echo "$out2" | tail -1)"
  rm -r "$empty"
else
  echo "note  part B not run (give --logs DIR)"
fi
echo "RESULT: $([ $fail = 0 ] && echo PASS || echo FAIL)"
exit $fail
