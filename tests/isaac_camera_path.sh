#!/usr/bin/env bash
# Isaac Sim spike, stage F: a camera image stream from Isaac Sim to the agent container, over the management network and not over the radio.
#   COSIM_ACCEPT_ISAAC_EULA=Y tests/isaac_camera_path.sh [--config YAML] [--timeout S]     (images first: python3 orchestrator/cosim.py build --config YAML)
# Part A (no Docker): the frame protocol and the receiver, orchestrator/camera_sink.py
#   A1. three frames sent over TCP give three CSV rows with seq, width, height, nbytes and an arrival time
#   A2. a constant frame has std 0 and a varied one std > 1.0 (byte sample)
#   A3. a frame whose nbytes is not width x height x 3 is flagged size_ok = 0
#   A4. with ppm_every = 2 the receiver also writes frames 0, 2, 4 as binary PPM files (debugging aid; off by default)
# Part B (full stack, NVIDIA runtime and Isaac image needed): criteria F1-F4 of docs/EXPERIMENTS.md "Isaac Sim stage F", fixed before the first run:
#   F1 average >= 10 frames/s at the receiver from arming to the end of the flight window   F2 frame size correct and >= 90 % of frames not constant
#   F3 the ns-3 pcap holds frames and none to or from TCP port 5700 + px4_instance            F4 flight-window RTF >= time.min_rtf (tools/analyze_isaac.py)
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CFG="$ROOT/config/experiments/isaac_camera.yaml"; TIMEOUT=900
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
import csv, os, socket, sys, tempfile, threading, time
sys.path.insert(0, os.path.join(os.environ["ROOT"], "orchestrator"))
import camera_sink

d = tempfile.mkdtemp()
path = os.path.join(d, "cam.csv")
sink = camera_sink.Sink(("127.0.0.1", 0), path)
th = threading.Thread(target=sink.run, daemon=True); th.start()
c = socket.create_connection(("127.0.0.1", sink.port))
w, h = 8, 6
c.sendall(camera_sink.pack_frame(0, 1.0, w, h, bytes(w * h * 3)))                        # constant
c.sendall(camera_sink.pack_frame(1, 1.1, w, h, bytes((i * 37) % 256 for i in range(w * h * 3))))   # varied
c.sendall(camera_sink.pack_frame(2, 1.2, w, h, bytes(10)))                                # wrong size for 8 x 6
time.sleep(0.5)
sink.stop(); c.close(); th.join(timeout=2)
# A4 uses a second receiver that saves every 2nd frame as a binary PPM
d2 = tempfile.mkdtemp()
sink2 = camera_sink.Sink(("127.0.0.1", 0), os.path.join(d2, "cam.csv"), ppm_dir=d2, ppm_every=2)
th2 = threading.Thread(target=sink2.run, daemon=True); th2.start()
c2 = socket.create_connection(("127.0.0.1", sink2.port))
for i in range(5):
    c2.sendall(camera_sink.pack_frame(i, float(i), 2, 1, bytes([i, 0, 0, 0, i, 0])))
time.sleep(0.5); sink2.stop(); c2.close(); th2.join(timeout=2)
saved = sorted(f for f in os.listdir(d2) if f.endswith(".ppm"))
okp = saved == ["frame_000000.ppm", "frame_000002.ppm", "frame_000004.ppm"] and open(os.path.join(d2, saved[1]), "rb").read() == b"P6\n2 1\n255\n" + bytes([2, 0, 0, 0, 2, 0])
print("A4", "ok" if okp else f"saved {saved}")
rows = list(csv.DictReader(open(path)))
ok1 = len(rows) == 3 and [r["seq"] for r in rows] == ["0", "1", "2"] and all(float(r["t_wall_s"]) > 1e9 and r["width"] == "8" and r["height"] == "6" for r in rows)
print("A1", "ok" if ok1 else f"rows {rows}")
ok2 = len(rows) == 3 and float(rows[0]["std"]) == 0.0 and float(rows[1]["std"]) > 1.0
print("A2", "ok" if ok2 else f"std {[r.get('std') for r in rows]}")
ok3 = len(rows) == 3 and [r["size_ok"] for r in rows] == ["1", "1", "0"]
print("A3", "ok" if ok3 else f"size_ok {[r.get('size_ok') for r in rows]}")
E
)
for k in A1 A2 A3 A4; do
  line=$(echo "$a" | grep "^$k " | head -1)
  name=$(case $k in A1) echo "receiver logs one row per frame";; A2) echo "constant and varied frames are told apart";; A3) echo "a wrongly sized frame is flagged";; A4) echo "optional frame dump writes every Nth frame as PPM";; esac)
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
sleep 10
T_ARM=$(python3 -c 'import time; print(time.time())')
px commander arm >/dev/null; px commander takeoff >/dev/null
sleep 45
T_END=$(python3 -c 'import time; print(time.time())')
dc stop -t 25 ns3 >/dev/null 2>&1

python3 - "$OUT/camera_agent1.csv" "$T_ARM" "$T_END" <<'E' > "$OUT/camera_check.txt" 2>&1
import csv, sys
try:
    rows = list(csv.DictReader(open(sys.argv[1])))
except FileNotFoundError:
    rows = []
t0, t1 = float(sys.argv[2]), float(sys.argv[3])
fl = [r for r in rows if t0 <= float(r["t_wall_s"]) <= t1]
if not fl:
    print(f"F1 bad no frames between arming and the end ({len(rows)} frames in the file)"); sys.exit(0)
rate = len(fl) / (t1 - t0)
print(("F1 ok" if rate >= 10.0 else "F1 bad") + f" {len(fl)} frames in {t1 - t0:.1f} s = {rate:.1f} frames/s")
varied = sum(1 for r in fl if float(r["std"]) > 1.0) / len(fl)
sized = all(r["size_ok"] == "1" for r in fl)
print(("F2 ok" if sized and varied >= 0.9 else "F2 bad") + f" size ok for all {sized}, {100 * varied:.0f} % of frames not constant, {fl[0]['width']}x{fl[0]['height']}")
E
cat "$OUT/camera_check.txt" | sed 's/^/  /'
grep -q "^F1 ok" "$OUT/camera_check.txt" && chk "F1 average frame rate at the receiver >= 10/s" ok || chk "F1 average frame rate at the receiver >= 10/s" "$(grep '^F1' "$OUT/camera_check.txt")"
grep -q "^F2 ok" "$OUT/camera_check.txt" && chk "F2 frames correctly sized and mostly not constant" ok || chk "F2 frames correctly sized and mostly not constant" "$(grep '^F2' "$OUT/camera_check.txt")"

PCAP=$(ls "$OUT"/ns3/wifi-*.pcap 2>/dev/null | head -1)
if [ -n "$PCAP" ]; then
  all=$(tcpdump -r "$PCAP" -nn 2>/dev/null | wc -l); cam=$(tcpdump -r "$PCAP" -nn 'tcp port 5700' 2>/dev/null | wc -l)
  [ "$all" -gt 0 ] && [ "$cam" = 0 ] && chk "F3 ns-3 pcap has $all frames, none on TCP port 5700" ok || chk "F3 ns-3 pcap has frames and none on TCP port 5700" "$all frames, $cam on port 5700"
else
  chk "F3 ns-3 pcap present" "no pcap in $OUT/ns3"
fi

out=$(python3 "$ROOT/tools/analyze_isaac.py" "$OUT" "$CFG" 2>&1); echo "$out" | sed 's/^/  | /'
echo "$out" | grep -q "^PASS  RTF flight window" && chk "F4 flight-window RTF >= min_rtf with the camera on" ok || chk "F4 flight-window RTF >= min_rtf with the camera on" "$(echo "$out" | grep -E 'RTF flight|not certified' | head -1)"

echo "RESULT: $([ $fail = 0 ] && echo PASS || echo FAIL)"
exit $fail
