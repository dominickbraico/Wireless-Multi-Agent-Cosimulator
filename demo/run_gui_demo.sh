#!/usr/bin/env bash
# PX4 x500s flying leader-follower through the ns-3 Wi-Fi emulation (every agent in the experiment YAML), with the Gazebo GUI,
# for demonstrations and screen recordings.
#
#   demo/run_gui_demo.sh [--config YAML] [--world NAME] [--record] [--wireshark] [--no-gui] [--no-prompt]
#
#   --config Y  experiment YAML (default: config/experiments/two_agent_wifi.yaml). Every agent flies (leader and followers as
#               resolved by orchestrator/fleet.py). The overview camera frames every agent; demo/monitor.py is laid out for the two-agent config:
#               the dashboard shows only agent1 and agent2.
#   --world N   Gazebo world (worlds/N.sdf). Default: baylands (PX4's outdoor park-and-water world). px4_flat = plain ground.
#   (default)   starts everything, opens the Gazebo GUI, sets an overview camera, waits for you to press Enter
#               (start your screen recorder first), then flies. Needs sudo (real network namespaces).
#   --record    also record the whole screen with ffmpeg (x11grab) to the log dir. Needs ffmpeg: sudo apt install ffmpeg
#   --wireshark open Wireshark live on the two agents' TAP interfaces (swarm traffic only; saved to <logs>/interagent.pcapng).
#               Needs: sudo apt install wireshark. Capture runs under sudo (dumpcap), the Wireshark window runs as you.
#   --no-gui    do not open the GUI (headless run of the same flow)
#   --no-prompt do not wait for Enter (start the flight after a 5 s countdown)
#
# Env: SIDE (square side, default 14 m), ALT (default 6 m), SPEED (default 1.5 m/s), COSIM_ROOTLESS=1 (test the flow in the
# rootless sandbox instead of sudo; the GUI cannot attach in that mode, so combine with --no-gui).
set -u
# under plain sudo HOME may be /root; use the invoking user's home so tool paths resolve
[ -n "${SUDO_USER:-}" ] && HOME="$(getent passwd "$SUDO_USER" | cut -d: -f6)"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
GUI=1; RECORD=0; WS=0; PROMPT=1; WORLD=baylands; CFG_ARG=()
while [ $# -gt 0 ]; do case "$1" in --no-gui) GUI=0;; --wireshark) WS=1;; --record) RECORD=1;; --no-prompt) PROMPT=0;; --world) shift; WORLD="${1:-}";; --config) shift; CFG_ARG=(--config "${1:?--config needs a path}");; --config=*) CFG_ARG=(--config "${1#--config=}");; *) echo "unknown option $1"; exit 2;; esac; shift; done
OUT="$ROOT/logs/gui_demo_$(date +%Y%m%d_%H%M%S)"; mkdir -p "$OUT"
SIDE="${SIDE:-14.0}"; ALT="${ALT:-6.0}"; SPEED="${SPEED:-1.5}"
TESTPID=""; GUIPID=""; FFPID=""; MONPID=""; WSPID=""
# agent TAP interfaces, read from the experiment YAML (override with WS_IFACES)
CFG="${CFG_ARG[1]:-$ROOT/config/experiments/two_agent_wifi.yaml}"
[ -f "$CFG" ] || { echo "config not found: $CFG"; exit 2; }
WS_IFACES="${WS_IFACES:-$(python3 -c "import sys,yaml; print(' '.join(a['tap'] for a in yaml.safe_load(open(sys.argv[1]))['agents']))" "$CFG")}"
# Whatever happens (error, Ctrl-C), stop what we started: background jobs of a script do not get the terminal's Ctrl-C.
on_exit() {
  trap - INT TERM EXIT
  [ -n "$MONPID" ] && kill "$MONPID" 2>/dev/null
  [ -n "$WSPID" ] && sudo pkill -TERM -f "[d]umpcap -i ${WS_IFACES%% *}" 2>/dev/null   # Wireshark itself stays open with the capture
  [ -n "$FFPID" ] && { kill -INT "$FFPID" 2>/dev/null; wait "$FFPID" 2>/dev/null; }
  [ -n "$GUIPID" ] && kill "$GUIPID" 2>/dev/null
  if [ -n "$TESTPID" ] && kill -0 "$TESTPID" 2>/dev/null; then
    echo "stopping the simulation..."
    kill -TERM "$TESTPID" 2>/dev/null
    [ -z "${COSIM_ROOTLESS:-}" ] && sudo pkill -TERM -f "[l]eader_follower_fly" 2>/dev/null
    for _ in $(seq 1 40); do kill -0 "$TESTPID" 2>/dev/null || break; sleep 1; done
  fi
}
trap 'exit 130' INT TERM
trap on_exit EXIT
say() { printf '\n\033[1m== %s\033[0m\n' "$*"; }

# ---- preflight ----
say "Preflight"
miss=0
for f in "$HOME/dev/ns-3-build/ns-allinone-3.48/ns-3.48/build/scratch/wifi_adhoc_tap/ns3.48-wifi_adhoc_tap-optimized" \
         "$HOME/dev/px4-runtime/px4_sitl_default/bin/px4" "$HOME/dev/px4-runtime/MicroXRCEAgent" \
         "$HOME/dev/px4-runtime/ws/install/setup.bash" "$HOME/.simulation-gazebo/models/x500/model.sdf" \
         "$ROOT/ros2_ws/install/setup.bash" "$ROOT/worlds/$WORLD.sdf"; do
  [ -e "$f" ] || { echo "MISSING: $f"; miss=1; }
done
[ $miss = 0 ] || { echo "Build the missing pieces first (see docs/SETUP.md and tools/setup_px4_runtime.sh)."; exit 1; }
[ "$WORLD" = baylands ] && [ ! -d "$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/models/baylands" ] && echo "note: the Baylands model is not cached yet; the first start downloads it from Gazebo Fuel (needs internet, can take minutes)."
if [ $WS = 1 ] && [ -n "${COSIM_ROOTLESS:-}" ]; then echo "--wireshark is not available in rootless test mode"; WS=0; fi
if [ $WS = 1 ] && { ! command -v wireshark >/dev/null || ! command -v dumpcap >/dev/null; }; then echo "--wireshark needs Wireshark:  sudo apt install wireshark"; exit 1; fi
if [ $WS = 1 ] && ! id -nG | tr ' ' '\n' | grep -qx wireshark; then
  echo "--wireshark: your user is not in the 'wireshark' group, so the Wireshark window cannot start its capture helper (Permission denied)."
  echo "  sudo dpkg-reconfigure wireshark-common     (answer Yes)"; echo "  sudo usermod -aG wireshark $USER           then log out and back in"; exit 1
fi
if [ $RECORD = 1 ] && ! command -v ffmpeg >/dev/null; then echo "--record needs ffmpeg:  sudo apt install ffmpeg"; exit 1; fi
if pgrep -x px4 >/dev/null || pgrep -x MicroXRCEAgent >/dev/null || pgrep -f "[g]z sim" >/dev/null || ip link show br-mgmt >/dev/null 2>&1; then
  echo "Stale px4 / MicroXRCEAgent / gz sim processes are running (probably an interrupted earlier run). Stop them first:"
  echo "  $ROOT/demo/cleanup.sh"; exit 1
fi
echo "logs: $OUT"

# ---- start the stack (paused before the flight) ----
if [ -n "${COSIM_ROOTLESS:-}" ]; then RUN=("$ROOT/tests/run_rootless.sh" env); else
  sudo -v || exit 1                                     # ask for the password now, in the foreground
  RUN=(sudo env HOME="$HOME"); fi
say "Starting simulator (world: $WORLD), the PX4 vehicles, ns-3 and the pose bridge (about 30 s to 2 min)"
"${RUN[@]}" WAIT_FOR_GUI=1 PX4_WORLD="$WORLD" OUT="$OUT" SIDE="$SIDE" ALT="$ALT" SPEED="$SPEED" "$ROOT/tests/leader_follower_fly.sh" "${CFG_ARG[@]}" > "$OUT/test.out" 2>&1 &
TESTPID=$!
for i in $(seq 1 120); do grep -q "waiting on the ground" "$OUT/test.out" 2>/dev/null && break
  kill -0 $TESTPID 2>/dev/null || { echo "stack failed to start:"; tail -15 "$OUT/test.out"; exit 1; }; sleep 1; done
grep -q "waiting on the ground" "$OUT/test.out" || { echo "timed out starting the stack:"; tail -15 "$OUT/test.out"; exit 1; }
echo "all vehicles are up on the ground."

# ---- GUI: clean environment so ROS 2's gz stub / libraries cannot interfere ----
GZENV=(env -i HOME="$HOME" DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-}" XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}"
       WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" PATH=/usr/bin:/bin GZ_CONFIG_PATH=/usr/share/gz GZ_PARTITION=cosim GZ_IP=10.20.0.1
       GZ_SIM_RESOURCE_PATH="$HOME/.simulation-gazebo/models:$HOME/.simulation-gazebo/worlds")
if [ $GUI = 1 ]; then
  say "Opening the Gazebo GUI"
  "${GZENV[@]}" /usr/bin/gz sim -g > "$OUT/gui.log" 2>&1 &
  GUIPID=$!
  up=0
  for i in $(seq 1 60); do
    "${GZENV[@]}" timeout 4 /usr/bin/gz service -l 2>/dev/null | grep -q "^/gui/move_to/pose" && { up=1; break; }
    kill -0 $GUIPID 2>/dev/null || break; sleep 1; done
  if [ $up = 0 ]; then
    echo "The GUI did not connect to the simulator within 60 s. See $OUT/gui.log"; tail -5 "$OUT/gui.log"
    echo "Check:  ip addr show br-mgmt   (should show 10.20.0.1)   and that no other gz process uses partition 'cosim'."
  else
    echo "GUI connected."
    # top-down overview camera framing every agent in the YAML and the leader's square (PX4 would otherwise follow the leader only).
    # The longer side of the formation runs along the screen's wide axis; height assumes a 60 deg horizontal and a 16:9 window.
    read -r CX CY CZ QX QY QZ QW < <(python3 - "$CFG" "$SIDE" <<'PY'
import math, sys, yaml
cfg, side = yaml.safe_load(open(sys.argv[1])), float(sys.argv[2])
xs = [a["spawn_pose"][0] for a in cfg["agents"]]
ys = [a["spawn_pose"][1] for a in cfg["agents"]]
x0, x1, y0, y1 = min(xs), max(xs) + side, min(ys), max(ys) + side     # the square extends `side` from the leader's spawn
cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
ex, ey = x1 - x0, y1 - y0
yaw = 0.0 if ey >= ex else math.pi / 2          # yaw 0: screen-horizontal is world y; yaw 90 deg: world x
long_, short = (ey, ex) if ey >= ex else (ex, ey)
h = 1.15 * max(long_ / 2 / math.tan(math.radians(30)), short / 2 / 0.325)
h = max(h, 20.0)
pitch = math.pi / 2                             # straight down
cy_, sy_, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
print(cx, cy, h, -sy_ * sp, cy_ * sp, sy_ * cp, cy_ * cp)      # position, then x y z w for rotation Rz(yaw) * Ry(pitch)
PY
)
    "${GZENV[@]}" /usr/bin/gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 2000 --req 'data: ""' >/dev/null 2>&1
    "${GZENV[@]}" /usr/bin/gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 2000 \
        --req "pose: {position: {x: $CX, y: $CY, z: $CZ} orientation: {x: $QX, y: $QY, z: $QZ, w: $QW}}" >/dev/null 2>&1 \
        && echo "overview camera set (drag in the GUI to adjust)" || echo "could not set the camera automatically; adjust it by hand"
  fi
fi

# ---- Wireshark: live view of the inter-agent traffic on the TAPs (frames entering/leaving ns-3) ----
if [ $WS = 1 ]; then
  say "Opening Wireshark on: $WS_IFACES  (UDP 17900-18149 = swarm DDS domain; saved to $OUT/interagent.pcapng)"
  IFARGS=(); for i in $WS_IFACES; do IFARGS+=(-i "$i"); done
  ( sudo dumpcap "${IFARGS[@]}" -q -f "udp portrange 17900-18149" -w - 2>"$OUT/dumpcap.log" \
      | tee "$OUT/interagent.pcapng" | wireshark -k -i - -Y "rtps" >"$OUT/wireshark.log" 2>&1 ) &
  WSPID=$!
  sleep 3
  grep -qi "no such device\|error" "$OUT/dumpcap.log" 2>/dev/null && { echo "dumpcap problem:"; head -3 "$OUT/dumpcap.log"; }
fi

# ---- recording ----
FFPID=""
if [ $RECORD = 1 ]; then
  SZ=$(xdpyinfo | awk '/dimensions:/{print $2}')
  say "Recording the screen ($SZ) to $OUT/demo.mp4"
  ffmpeg -loglevel error -y -f x11grab -framerate 30 -video_size "$SZ" -i "${DISPLAY:-:0}" -c:v libx264 -preset ultrafast -crf 23 "$OUT/demo.mp4" &
  FFPID=$!; sleep 2
fi

# ---- start ----
if [ $PROMPT = 1 ] && [ $GUI = 1 ]; then
  say "Ready. Arrange the windows (Gazebo + a terminal running the dashboard), start your screen recorder, then press Enter."
  echo "Dashboard command for a second terminal:   \"$ROOT/demo/monitor.py\" \"$OUT\""
  read -r _
else
  say "Starting the flight in 5 s"; sleep 5
fi
touch "$OUT/go"
say "Flying: take-off, the leader flies a ${SIDE} m square, the follower tracks it over the emulated link (about 90 s)"
( "$ROOT/demo/monitor.py" "$OUT" ) 2>/dev/null &
MONPID=$!
wait $TESTPID
kill $MONPID 2>/dev/null
[ -n "$FFPID" ] && { kill -INT $FFPID 2>/dev/null; wait $FFPID 2>/dev/null; echo "video: $OUT/demo.mp4"; }
[ -n "${GUIPID:-}" ] && kill $GUIPID 2>/dev/null
say "Finished. Checks:"
grep -E "^(PASS|FAIL)|formation error|swarm link" "$OUT/test.out" | cut -c1-170
echo; echo "Plot: $OUT/formation.png      all logs: $OUT"
if [ -z "${COSIM_ROOTLESS:-}" ]; then echo "(files were written by root; if needed:  sudo chown -R $USER \"$OUT\" ~/dev/px4-runtime)"; fi
exit 0
