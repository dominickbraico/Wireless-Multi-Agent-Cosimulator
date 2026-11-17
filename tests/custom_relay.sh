#!/usr/bin/env bash
# Checks the radio relay (ros2_ws/src/radio_relay) on loopback: no network emulation, no PX4, no Docker.
# Two relays stand in for two agents: A bridges local domain 81, B bridges local domain 82, both use "radio" domain 83.
#   R1  a message published on a listed topic in A's local domain arrives in B's local domain, unchanged
#   R2  no echo: 100 messages published in A are received exactly 100 times in A and in B; A logs 100 tx and 0 rx rows
#   R2b the same payloads published on both sides at once: each side receives exactly 100 (its own 50 and the peer's 50)
#   R3  a topic that is not listed does not cross
#   R4  the QoS in effect on the radio side is the one in the topics file
#   R4b reliable + transient_local: restarting B does not make A send again
#   E1  an unknown message type is refused with exit code 2 and a message naming it
#   R5  invalid `scenario.custom` sections are refused before anything starts, naming the key
#   A1  tools/analyze_radio.py matches sent and received messages by payload
#   C1  the template controller converges through three relays (skipped unless examples/template_ws is built and sourced)
# The count checks use a reliable topic: they test the relay's forwarding logic, not loss.
# Needs the workspace built and sourced:  (cd ros2_ws && colcon build --symlink-install) && source ros2_ws/install/setup.bash
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT=${OUT:-$ROOT/logs/custom_relay_$(date +%Y%m%d_%H%M%S)}; mkdir -p "$OUT"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
PROBE="$ROOT/tests/_relay_probe.py"
T=/relay_test/chatter
FAIL=0
check() { if [ "$2" = ok ]; then echo "PASS  $1"; else echo "FAIL  $1  ($3)"; FAIL=1; fi; }
PGIDS=""
start() { setsid "$@" & PGIDS="$PGIDS $!"; LAST=$!; }           # each helper in its own session: ros2 run leaves its child behind
stop() { kill -INT -- -"$1" 2>/dev/null; sleep 0.5; kill -KILL -- -"$1" 2>/dev/null; }
cleanup() { for p in $PGIDS; do kill -KILL -- -"$p" 2>/dev/null; done; }
trap cleanup EXIT
relay() { # relay <tag> <local domain> <topics file>
  start ros2 run radio_relay radio_relay --ros-args -p topics_file:="$3" -p agent_name:="$1" -p local_domain:="$2" -p radio_domain:=83 \
      -p log_csv:="$OUT/relay_$1.csv" > "$OUT/relay_$1.log" 2>&1
}
pub() { ROS_DOMAIN_ID=$1 python3 "$PROBE" pub "${@:2}"; }      # pub <domain> <topic> <count> <prefix> <wait_subs> [qos]
sub() { ROS_DOMAIN_ID=$1 python3 "$PROBE" sub "${@:2}"; }      # sub <domain> <topic> <seconds> [qos]
rows() { awk -F, -v d="$2" -v t="$3" 'NR>1 && $2==d && $3==t' "$1" 2>/dev/null | wc -l; }

if ! ros2 pkg prefix radio_relay >/dev/null 2>&1; then
  for n in R1 R2 R2b R3 R4 R4b E1; do check "$n" bad "package 'radio_relay' not found: build and source ros2_ws"; done; exit 1
fi

# ---- R1, R2, R2b, R3: one listed reliable topic
printf '%s std_msgs/msg/String reliable volatile 200\n' "$T" > "$OUT/topics.txt"
relay A 81 "$OUT/topics.txt"; RA=$LAST
relay B 82 "$OUT/topics.txt"; RB=$LAST
sleep 3

sub 82 $T 14 > "$OUT/r1_b.txt" & S1=$!
sub 81 $T 14 > "$OUT/r1_a.txt" & S2=$!
sub 82 /relay_test/unlisted 14 > "$OUT/r3_b.txt" & S3=$!
sleep 3
pub 81 /relay_test/unlisted 5 nope 0 & P1=$!
pub 81 $T 100 msg 2
wait $P1 $S1 $S2 $S3 2>/dev/null
exp=$(seq 0 99 | sed 's/^/msg-/' | sort)
[ "$(sort "$OUT/r1_b.txt")" = "$exp" ] && check "R1 listed topic crosses unchanged (100 distinct strings)" ok || check "R1 listed topic crosses unchanged" bad "$(wc -l < "$OUT/r1_b.txt") lines in $OUT/r1_b.txt"
na=$(wc -l < "$OUT/r1_a.txt"); nb=$(wc -l < "$OUT/r1_b.txt"); tx=$(rows "$OUT/relay_A.csv" tx $T); rx=$(rows "$OUT/relay_A.csv" rx $T)
[ "$na" = 100 ] && [ "$nb" = 100 ] && [ "$tx" = 100 ] && [ "$rx" = 0 ] && check "R2 no echo (A heard 100, B heard 100, relay A tx 100 rx 0)" ok || check "R2 no echo" bad "A heard $na, B heard $nb, relay A tx $tx rx $rx"
[ ! -s "$OUT/r3_b.txt" ] && check "R3 unlisted topic does not cross" ok || check "R3 unlisted topic does not cross" bad "$(wc -l < "$OUT/r3_b.txt") messages arrived"

sub 81 $T 14 > "$OUT/r2b_a.txt" & S1=$!
sub 82 $T 14 > "$OUT/r2b_b.txt" & S2=$!
sleep 3
pub 82 $T 50 same 2 & P1=$!
pub 81 $T 50 same 2
wait $P1 $S1 $S2 2>/dev/null      # never a bare wait: the relays are background jobs too
na=$(wc -l < "$OUT/r2b_a.txt"); nb=$(wc -l < "$OUT/r2b_b.txt")
[ "$na" = 100 ] && [ "$nb" = 100 ] && check "R2b identical payloads from both sides: each side receives 100" ok || check "R2b identical payloads from both sides" bad "A heard $na, B heard $nb"
stop $RA; stop $RB

# ---- R4, R4b: reliable + transient_local, depth 3
printf '%s std_msgs/msg/String reliable transient_local 3\n' "$T" > "$OUT/topics_tl.txt"
relay A 81 "$OUT/topics_tl.txt"; RA=$LAST
relay B 82 "$OUT/topics_tl.txt"; RB=$LAST
sleep 3
ROS_DOMAIN_ID=83 ros2 topic info -v $T > "$OUT/r4_info.txt" 2>&1
grep -q "RELIABLE TRANSIENT_LOCAL KEEP_LAST depth=3" "$OUT/relay_A.log" && grep -q "Reliability: RELIABLE" "$OUT/r4_info.txt" && grep -q "Durability: TRANSIENT_LOCAL" "$OUT/r4_info.txt" \
  && ! grep -q "Reliability: BEST_EFFORT" "$OUT/r4_info.txt" && check "R4 radio-side QoS is the one in the topics file (reliable, transient_local, depth 3)" ok \
  || check "R4 radio-side QoS" bad "$OUT/r4_info.txt, $OUT/relay_A.log"
pub 81 $T 5 tl 1 reliable transient_local 3
sleep 1; before=$(rows "$OUT/relay_A.csv" tx $T)
stop $RB; relay B 82 "$OUT/topics_tl.txt"; RB=$LAST
sleep 5; after=$(rows "$OUT/relay_A.csv" tx $T)
[ "$before" = 5 ] && [ "$after" = 5 ] && check "R4b restarting B does not make A send again (tx rows 5 before and after)" ok || check "R4b restart does not resend" bad "tx rows before $before, after $after"
stop $RA; stop $RB

# ---- E1: unknown type
printf '%s no_such_pkg/msg/Nope reliable volatile 10\n' "$T" > "$OUT/topics_bad.txt"
timeout 20 ros2 run radio_relay radio_relay --ros-args -p topics_file:="$OUT/topics_bad.txt" > "$OUT/e1.log" 2>&1; rc=$?
[ "$rc" = 2 ] && grep -q "no_such_pkg/msg/Nope" "$OUT/e1.log" && check "E1 unknown message type refused (exit 2, type named)" ok || check "E1 unknown message type refused" bad "exit $rc, $OUT/e1.log"

# ---- R5: the experiment YAML is validated before anything starts (orchestrator/custom_config.py); needs no ROS
WS="$OUT/r5 ws"; mkdir -p "$WS/src/demo_pkg/launch"      # a path with a space on purpose
echo '<package format="3"><name>demo_pkg</name></package>' > "$WS/src/demo_pkg/package.xml"; : > "$WS/src/demo_pkg/launch/agent.launch.py"
r5() { # r5 <label> <text the message must contain, or OK> <python statement that edits c["scenario"]["custom"] as x>
  python3 - "$ROOT/config/experiments/four_agent_mesh_dropout.yaml" "$OUT/r5.yaml" "$WS" "$3" <<'PY' || return
import sys, yaml
c = yaml.safe_load(open(sys.argv[1]))
c["scenario"] = {"type": "custom", "custom": {"workspace": sys.argv[3], "launch": {"package": "demo_pkg", "file": "agent.launch.py"},
                 "radio_topics": [{"name": "/swarm/state", "type": "std_msgs/msg/String"}]}}
x = c["scenario"]["custom"]
exec(sys.argv[4])
yaml.safe_dump(c, open(sys.argv[2], "w"))
PY
  msg=$(python3 "$ROOT/orchestrator/custom_config.py" "$OUT/r5.yaml" 2>&1 >/dev/null); rc=$?
  if [ "$2" = OK ]; then [ $rc = 0 ] && check "R5 $1" ok || check "R5 $1" bad "exit $rc: $msg"
  else [ $rc = 2 ] && echo "$msg" | grep -qF "$2" && check "R5 $1 refused" ok || check "R5 $1 refused" bad "exit $rc: $msg"; fi
}
r5 "valid configuration accepted (workspace path with a space)" OK "pass"
r5 "missing workspace" "scenario.custom.workspace" "del x['workspace']"
r5 "workspace without src/" "src" "x['workspace'] = '/tmp'"
r5 "launch package not in the workspace" "scenario.custom.launch.package" "x['launch']['package'] = 'nope'"
r5 "launch file not in the package" "scenario.custom.launch.file" "x['launch']['file'] = 'nope.launch.py'"
r5 "empty radio_topics" "scenario.custom.radio_topics" "x['radio_topics'] = []"
r5 "duplicate topic" "duplicate" "x['radio_topics'] = x['radio_topics'] * 2"
r5 "relative topic name" "absolute" "x['radio_topics'][0]['name'] = 'swarm/state'"
r5 "PX4 topic /fmu/out/vehicle_status" "PX4" "x['radio_topics'][0]['name'] = '/fmu/out/vehicle_status'"
r5 "PX4 topic /px4_1/fmu/in/trajectory_setpoint" "PX4" "x['radio_topics'][0]['name'] = '/px4_1/fmu/in/trajectory_setpoint'"
r5 "bad reliability" "reliability" "x['radio_topics'][0]['reliability'] = 'bogus'"
r5 "depth 0" "depth" "x['radio_topics'][0]['depth'] = 0"
r5 "bad type" "type" "x['radio_topics'][0]['type'] = 'String'"

# ---- A1: tools/analyze_radio.py on hand-written relay logs (10 sent, 9 received, 4 ms apart; one payload repeated)
mkdir -p "$OUT/a1/relay"
{ echo "t_s,direction,topic,bytes,hash"; for i in $(seq 0 9); do echo "100.$((i))00000,tx,/t,8,$( [ $i -lt 2 ] && echo same || echo h$i )"; done; echo "103.000000,tx,/t,8,end"; } > "$OUT/a1/relay/relay_agent1.csv"
{ echo "t_s,direction,topic,bytes,hash"; for i in 0 1 2 3 4 5 6 7 9; do echo "100.$((i))04000,rx,/t,8,$( [ $i -lt 2 ] && echo same || echo h$i )"; done; } > "$OUT/a1/relay/relay_agent2.csv"
a1=$(python3 "$ROOT/tools/analyze_radio.py" "$OUT/a1" --min-delivery 0.95); rc=$?
echo "$a1" | grep -q "agent1->agent2: sent=10 received=9 delivery=0.900 latency_ms p50=4.00" && [ $rc = 1 ] \
  && check "A1 analyzer: 9 of 10 matched by payload, latency 4 ms, below --min-delivery gives exit 1" ok || check "A1 analyzer" bad "exit $rc: $a1"

# ---- C1: the template controller (examples/template_ws) without vehicles: three agents, each with its own local domain and relay
if ros2 pkg prefix consensus_controller >/dev/null 2>&1; then
  printf '/swarm/state consensus_msgs/msg/AgentState best_effort volatile 10\n' > "$OUT/topics_c1.txt"
  mkdir -p "$OUT/c1/relay"; CP=""
  i=0; for alt in 4.0 6.0 8.0; do i=$((i+1))
    start ros2 run radio_relay radio_relay --ros-args -p topics_file:="$OUT/topics_c1.txt" -p agent_name:=agent$i -p local_domain:=$((83+i)) -p radio_domain:=88 \
        -p log_csv:="$OUT/c1/relay/relay_agent$i.csv" > "$OUT/c1/relay_agent$i.log" 2>&1
  done
  sleep 3
  i=0; for alt in 4.0 6.0 8.0; do i=$((i+1))
    ROS_DOMAIN_ID=$((83+i)) setsid ros2 launch consensus_controller agent.launch.py agent_id:=$i px4_instance:=$((i-1)) start_altitude:=$alt run_s:=20.0 \
        dry_run:=true log_dir:="$OUT/c1/agent$i" > "$OUT/c1/launch_agent$i.log" 2>&1 & CP="$CP $!"; PGIDS="$PGIDS $!"
  done
  wait $CP 2>/dev/null
  spread=$(python3 - "$OUT/c1" <<'PY'
import csv, sys
last = [float(list(csv.DictReader(open(f"{sys.argv[1]}/agent{i}/consensus.csv")))[-1]["setpoint_m"]) for i in (1, 2, 3)]
print(f"{max(last) - min(last):.3f} {sum(last) / 3:.2f}")
PY
)
  python3 -c "import sys; sys.exit(0 if float('${spread%% *}' or 9) < 0.1 else 1)" 2>/dev/null \
    && check "C1 template controller: three agents from 4, 6, 8 m agree within 0.1 m after 20 s (spread and mean: $spread)" ok || check "C1 template controller converges" bad "spread and mean: '$spread', $OUT/c1"
  c1=$(python3 "$ROOT/tools/analyze_radio.py" "$OUT/c1" --min-delivery 0.95); rc=$?
  [ $rc = 0 ] && [ "$(echo "$c1" | grep -c '^radio /swarm/state')" = 6 ] && check "C1 analyzer on the relay logs: 6 ordered pairs, delivery at least 0.95 on loopback" ok || check "C1 analyzer on the relay logs" bad "exit $rc: $c1"
  echo "$c1" > "$OUT/c1/radio.txt"
else
  echo "SKIP  C1 template controller (build and source examples/template_ws)"
fi

echo "logs: $OUT"; exit $FAIL
