#!/usr/bin/env bash
# The containerized leader-follower flight with the Gazebo GUI and a LIVE 2D map of every agent pair's signal strength.
# The simulation runs in Docker (one container per agent, ns-3, simulator); the Gazebo GUI and the map window run on the host.
#
#   demo/run_container_demo.sh [--config YAML] [--world NAME] [--no-gui] [--no-map] [--no-prompt]
#
#   --config Y   experiment YAML (default: config/experiments/five_agent_wifi.yaml)
#   --world N    Gazebo world (default: from the YAML; only worlds without downloads work in containers)
#   --no-gui     do not open the Gazebo GUI           --no-map  do not open the live map
#   --no-prompt  start the flight after a 5 s countdown instead of waiting for Enter
# Needs: Docker with the images built (python3 orchestrator/cosim.py build), a desktop session, Gazebo Harmonic on the host
# (for the GUI) and python3 with matplotlib + Tk (for the map). No sudo.
# Env: SIDE (square side, default 14 m), ALT (default 6 m), SPEED (default 1.5 m/s)
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
GUI=1; MAP=1; PROMPT=1; WORLD=""; CFG="$ROOT/config/experiments/five_agent_wifi.yaml"
while [ $# -gt 0 ]; do case "$1" in
  --no-gui) GUI=0;; --no-map) MAP=0;; --no-prompt) PROMPT=0;;
  --world) shift; WORLD="${1:?--world needs a name}";;
  --config) shift; CFG="${1:?--config needs a path}";; --config=*) CFG="${1#--config=}";;
  *) echo "unknown option $1"; exit 2;; esac; shift; done
[ -f "$CFG" ] || { echo "config not found: $CFG"; exit 2; }
CFG="$(cd "$(dirname "$CFG")" && pwd)/$(basename "$CFG")"
OUT="$ROOT/logs/container_demo_$(date +%Y%m%d_%H%M%S)"; mkdir -p "$OUT"
SIDE="${SIDE:-14.0}"; ALT="${ALT:-6.0}"; SPEED="${SPEED:-1.5}"
TESTPID=""; GUIPID=""; MAPPID=""
say() { printf '\n\033[1m== %s\033[0m\n' "$*"; }
on_exit() {
  trap - INT TERM EXIT
  [ -n "$MAPPID" ] && kill "$MAPPID" 2>/dev/null
  [ -n "$GUIPID" ] && kill "$GUIPID" 2>/dev/null
  if [ -n "$TESTPID" ] && kill -0 "$TESTPID" 2>/dev/null; then
    echo "stopping the simulation..."; kill -TERM "$TESTPID" 2>/dev/null
    for _ in $(seq 1 60); do kill -0 "$TESTPID" 2>/dev/null || break; sleep 1; done
  fi
}
trap 'exit 130' INT TERM
trap on_exit EXIT

say "Preflight"
docker image inspect cosim-agent cosim-ns3 cosim-sim-gazebo >/dev/null 2>&1 || { echo "Images missing. Build them:  python3 $ROOT/orchestrator/cosim.py build"; exit 1; }
[ -n "${DISPLAY:-}" ] || { echo "No DISPLAY: the GUI and the map need a desktop session (use --no-gui --no-map for a headless run)."; GUI=0; MAP=0; }
if [ $GUI = 1 ]; then
  [ -x /usr/bin/gz ] || { echo "The Gazebo GUI needs Gazebo Harmonic on the host (/usr/bin/gz). Use --no-gui, or install it (docs/SETUP.md)."; exit 1; }
  [ -d "$HOME/.simulation-gazebo/models/x500" ] || { echo "PX4's Gazebo models are missing on the host: run px4/fetch_gazebo_models.sh"; exit 1; }
fi
if [ $MAP = 1 ]; then python3 -c "import matplotlib, tkinter" 2>/dev/null || { echo "The live map needs matplotlib and Tk (sudo apt install python3-tk). Use --no-map to skip."; exit 1; }; fi
# host-side address on the management network: the Docker bridge's gateway (see orchestrator/containers.py)
GWIP="$(python3 "$ROOT/orchestrator/containers.py" "$CFG" | python3 -c "import json,sys; print(json.load(sys.stdin)['mgmt']['gateway'])")"
GZPART="$(python3 -c "import yaml,sys; print(yaml.safe_load(open(sys.argv[1]))['simulator']['gz_partition'])" "$CFG")"
echo "logs: $OUT   (host reaches the simulator from $GWIP, gz partition $GZPART)"

say "Starting the containers: simulator, ns-3 and one per agent (about 1 to 3 min)"
WAIT_FOR_GUI=1 OUT="$OUT" SIDE="$SIDE" ALT="$ALT" SPEED="$SPEED" ${WORLD:+PX4_WORLD="$WORLD"} \
  "$ROOT/tests/containers_leader_follower.sh" --config "$CFG" > "$OUT/test.out" 2>&1 &
TESTPID=$!
for i in $(seq 1 400); do grep -q "waiting on the ground" "$OUT/test.out" 2>/dev/null && break
  kill -0 $TESTPID 2>/dev/null || { echo "stack failed to start:"; tail -20 "$OUT/test.out"; exit 1; }; sleep 1; done
grep -q "waiting on the ground" "$OUT/test.out" || { echo "timed out starting the stack:"; tail -15 "$OUT/test.out"; exit 1; }
echo "all vehicles are up on the ground."

# The map is opened only now: it competes for CPU, and PX4's start-up is the real-time-sensitive part of a run.
if [ $MAP = 1 ]; then
  python3 "$ROOT/tools/plot_comm_map.py" "$OUT" --live > "$OUT/map.log" 2>&1 &
  MAPPID=$!; echo "live map window opened"
fi

if [ $GUI = 1 ]; then
  say "Opening the Gazebo GUI (on the host, attached to the simulator container)"
  # clean environment so ROS 2's gz stub / libraries cannot interfere
  GZENV=(env -i HOME="$HOME" DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-}" XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}"
         WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" PATH=/usr/bin:/bin GZ_CONFIG_PATH=/usr/share/gz GZ_PARTITION="$GZPART" GZ_IP="$GWIP"
         GZ_SIM_RESOURCE_PATH="$HOME/.simulation-gazebo/models:$HOME/.simulation-gazebo/worlds")
  "${GZENV[@]}" /usr/bin/gz sim -g > "$OUT/gui.log" 2>&1 &
  GUIPID=$!
  up=0
  for i in $(seq 1 60); do
    "${GZENV[@]}" timeout 4 /usr/bin/gz service -l 2>/dev/null | grep -q "^/gui/move_to/pose" && { up=1; break; }
    kill -0 $GUIPID 2>/dev/null || break; sleep 1; done
  if [ $up = 0 ]; then
    echo "The GUI did not connect to the simulator within 60 s. See $OUT/gui.log"; tail -5 "$OUT/gui.log"
    echo "Check that the host can reach the simulator:  ping -c1 $(python3 "$ROOT/orchestrator/containers.py" "$CFG" | python3 -c "import json,sys; print(json.load(sys.stdin)['mgmt']['sim_ip'])")"
  else
    echo "GUI connected."
    read -r CX CY CZ QX QY QZ QW < <(python3 "$ROOT/demo/overview_camera.py" "$CFG" "$SIDE")
    "${GZENV[@]}" /usr/bin/gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 2000 --req 'data: ""' >/dev/null 2>&1
    "${GZENV[@]}" /usr/bin/gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 2000 \
        --req "pose: {position: {x: $CX, y: $CY, z: $CZ} orientation: {x: $QX, y: $QY, z: $QZ, w: $QW}}" >/dev/null 2>&1 \
        && echo "overview camera set (drag in the GUI to adjust)" || echo "could not set the camera automatically; adjust it by hand"
  fi
fi

if [ $PROMPT = 1 ]; then
  say "Ready. Arrange the Gazebo and map windows side by side (start your screen recorder), then press Enter to fly."
  read -r _
else
  say "Starting the flight in 5 s"; sleep 5
fi
touch "$OUT/go"
say "Flying: take-off, the leader flies a ${SIDE} m square, the followers track it over the emulated link (about 90 s)"
wait $TESTPID; RC=$?
TESTPID=""
say "Finished. Checks:"
grep -E "^(PASS|FAIL)|formation error|swarm link" "$OUT/test.out" | cut -c1-170
if [ $GUI = 1 ] && grep -q "^FAIL  Gazebo RTF" "$OUT/test.out"; then
  echo; echo "Note on the RTF line: attaching the Gazebo GUI makes the simulator pause for a moment while it sends the scene. That pause is before take-off and can pull"
  echo "the strict 5-s sliding mean under the threshold; the other checks cover the flight. Sim times where the real-time factor dipped under 0.5 (attach was at the"
  echo "stall nearest your GUI start; take-off is after Enter):  $(awk -F, 'NR>1 && $3<0.5 {printf "%d ", $2}' "$OUT/bridge/rtf.csv")"
  echo "A strict validity run is the headless one:  tests/containers_leader_follower.sh --config $CFG"
fi
echo; echo "Final map: $OUT/comm_map.png     plots: $OUT/formation.png     all logs: $OUT"
if [ $PROMPT = 1 ] && { [ -n "$GUIPID" ] || [ -n "$MAPPID" ]; }; then
  echo "The Gazebo and map windows stay open. Press Enter to close them."; read -r _
fi
exit $RC
