#!/usr/bin/env bash
# Isaac Sim spike, stage C: the `isaac` simulator driver. One agent container, PX4 SITL attached to Isaac Sim + Pegasus over the simulator MAVLink interface.
#   COSIM_ACCEPT_ISAAC_EULA=Y tests/containers_isaac_ready.sh [--config YAML] [--timeout S]     (images first: python3 orchestrator/cosim.py build --config YAML)
# Needs the NVIDIA Container Toolkit and the nvcr.io/nvidia/isaac-sim:5.1.0 image (docs/CONTAINERS.md, "Isaac Sim"). Not a supported simulator yet.
# Checks (criteria fixed before the first run, docs/EXPERIMENTS.md "Isaac Sim stage C"):
#   1. a vehicle with no Isaac definition (r1_rover) is refused with its name
#   2. without the NVIDIA runtime the generator stops with a message naming the NVIDIA Container Toolkit
#   3. without COSIM_ACCEPT_ISAAC_EULA=Y the generator stops with a message naming that variable
#   4. the generated Compose file gives a GPU to the simulator service and to no other service
#   5. `cosim up` brings the stack up and agent1 reports PX4 ready
#   6. the environment of the running px4 process has PX4_SIM_MODEL=none_iris and no PX4_GZ_ variable (read from /proc), and the simulator service environment no GZ_ variable
#   7. a camera block without width, height or rate_hz (or with a non-positive value) is refused with a message naming `camera.<key>`
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

# Checks 1-4 call the generator in-process, with the Docker runtime query and the EULA variable controlled by the test.
py() { PYTHONPATH="$ROOT/orchestrator" CFG="$CFG" OUTD="$OUT/gen" python3 - "$@"; }

r=$(py <<'E' 2>&1
import os, yaml, compose_gen
from unittest import mock
import simulators
cfg = yaml.safe_load(open(os.environ["CFG"]))
cfg["agents"][0]["vehicle"] = "r1_rover"
os.environ["COSIM_ACCEPT_ISAAC_EULA"] = "Y"
with mock.patch.object(simulators, "_docker_runtimes", return_value=["nvidia", "runc"]):
    try:
        compose_gen.generate(cfg, os.environ["OUTD"] + "1")
        print("NO ERROR")
    except SystemExit as e:
        print(e)
E
)
case "$r" in *r1_rover*) chk "vehicle without an Isaac definition is refused by name" ok;; *) chk "vehicle without an Isaac definition is refused by name" "$r";; esac

r=$(py <<'E' 2>&1
import os, yaml, compose_gen
from unittest import mock
import simulators
cfg = yaml.safe_load(open(os.environ["CFG"]))
os.environ["COSIM_ACCEPT_ISAAC_EULA"] = "Y"
with mock.patch.object(simulators, "_docker_runtimes", return_value=["runc"]):
    try:
        compose_gen.generate(cfg, os.environ["OUTD"] + "2")
        print("NO ERROR")
    except SystemExit as e:
        print(e)
E
)
case "$r" in *"NVIDIA Container Toolkit"*) chk "no NVIDIA runtime: message names the NVIDIA Container Toolkit" ok;; *) chk "no NVIDIA runtime: message names the NVIDIA Container Toolkit" "$r";; esac

r=$(py <<'E' 2>&1
import os, yaml, compose_gen
from unittest import mock
import simulators
cfg = yaml.safe_load(open(os.environ["CFG"]))
os.environ.pop("COSIM_ACCEPT_ISAAC_EULA", None)
with mock.patch.object(simulators, "_docker_runtimes", return_value=["nvidia", "runc"]):
    try:
        compose_gen.generate(cfg, os.environ["OUTD"] + "3")
        print("NO ERROR")
    except SystemExit as e:
        print(e)
E
)
case "$r" in *COSIM_ACCEPT_ISAAC_EULA*) chk "EULA not accepted: message names COSIM_ACCEPT_ISAAC_EULA" ok;; *) chk "EULA not accepted: message names COSIM_ACCEPT_ISAAC_EULA" "$r";; esac

CAMCFG="$ROOT/config/experiments/isaac_camera.yaml"
for key in width height rate_hz; do
  r=$(PYTHONPATH="$ROOT/orchestrator" CAMCFG="$CAMCFG" KEY=$key python3 - <<'E' 2>&1
import os, yaml, vehicles
cfg = yaml.safe_load(open(os.environ["CAMCFG"]))
del cfg["agents"][0]["camera"][os.environ["KEY"]]
try:
    vehicles.check(cfg); print("NO ERROR")
except SystemExit as e:
    print(e)
E
)
  case "$r" in *"camera.$key"*) chk "7 camera without $key is refused naming camera.$key" ok;; *) chk "7 camera without $key is refused naming camera.$key" "$r";; esac
done
r=$(PYTHONPATH="$ROOT/orchestrator" CAMCFG="$CAMCFG" python3 - <<'E' 2>&1
import os, yaml, vehicles
cfg = yaml.safe_load(open(os.environ["CAMCFG"]))
cfg["agents"][0]["camera"]["rate_hz"] = 0
try:
    vehicles.check(cfg); print("NO ERROR")
except SystemExit as e:
    print(e)
E
)
case "$r" in *"camera.rate_hz"*) chk "7 camera with rate_hz 0 is refused naming camera.rate_hz" ok;; *) chk "7 camera with rate_hz 0 is refused naming camera.rate_hz" "$r";; esac

if [ "${COSIM_ACCEPT_ISAAC_EULA:-}" != Y ]; then
  chk "checks 4-6 need COSIM_ACCEPT_ISAAC_EULA=Y (NVIDIA Omniverse licence agreement, accepted by you)" "variable not set"
  echo "RESULT: FAIL"; exit 1
fi

r=$(py <<'E' 2>&1
import os, yaml, compose_gen
from unittest import mock
import simulators
cfg = yaml.safe_load(open(os.environ["CFG"]))
with mock.patch.object(simulators, "_docker_runtimes", return_value=["nvidia", "runc"]):
    p = compose_gen.generate(cfg, os.environ["OUTD"] + "4")
svc = yaml.safe_load(open(p))["services"]
has = sorted(n for n, s in svc.items() if "devices" in str(s.get("deploy", {})))
print("GPU on:", ",".join(has))
sim_env = svc["simulator"]["environment"]
print("GZ vars:", ",".join(k for k in sim_env if k.startswith("GZ_")) or "none")
E
)
echo "$r" | grep -q "^GPU on: simulator$" && chk "GPU reserved for the simulator service only" ok || chk "GPU reserved for the simulator service only" "$r"
echo "$r" | grep -q "^GZ vars: none$" && chk "no GZ_ variable in the simulator service" ok || chk "no GZ_ variable in the simulator service" "$r"

echo "logs: $OUT"
cosim up --config "$CFG" --out "$OUT" --timeout "$TIMEOUT" > "$OUT.up.log" 2>&1; rc=$?
mv "$OUT.up.log" "$OUT/up.log"
[ $rc = 0 ] && [ -f "$OUT/agent1.px4_ready" ] && chk "cosim up: stack healthy and agent1 reports PX4 ready" ok || chk "cosim up: stack healthy and agent1 reports PX4 ready" "exit $rc; see $OUT/up.log"
dc() { docker compose -f "$OUT/compose/docker-compose.yml" "$@"; }
penv=$(dc exec -T agent1 sh -c 'for d in /proc/[0-9]*; do if [ "$(cat $d/comm 2>/dev/null)" = px4 ]; then tr "\0" "\n" < $d/environ; break; fi; done' 2>&1)
if [ -z "$penv" ]; then
  chk "6 environment of the running px4 process could be read" "no px4 process or empty environment"
else
  echo "$penv" | grep -q "^PX4_SIM_MODEL=none_iris$" && chk "6a px4 process has PX4_SIM_MODEL=none_iris" ok || chk "6a px4 process has PX4_SIM_MODEL=none_iris" "$(echo "$penv" | grep '^PX4_SIM' | tr '\n' ' ')"
  echo "$penv" | grep -q "^PX4_GZ_" && chk "6b px4 process has no PX4_GZ_ variable" "found $(echo "$penv" | grep '^PX4_GZ_' | tr '\n' ' ')" || chk "6b px4 process has no PX4_GZ_ variable" ok
fi

echo "RESULT: $([ $fail = 0 ] && echo PASS || echo FAIL)"
exit $fail
