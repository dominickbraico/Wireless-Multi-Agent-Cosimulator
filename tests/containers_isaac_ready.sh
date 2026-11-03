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
#   6. the PX4 environment has no PX4_GZ_ variable and the simulator service environment no GZ_ variable
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
with mock.patch.object(simulators, "_docker_runtimes", return_value=["nvidia", "runc"], create=True):
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
with mock.patch.object(simulators, "_docker_runtimes", return_value=["runc"], create=True):
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
with mock.patch.object(simulators, "_docker_runtimes", return_value=["nvidia", "runc"], create=True):
    try:
        compose_gen.generate(cfg, os.environ["OUTD"] + "3")
        print("NO ERROR")
    except SystemExit as e:
        print(e)
E
)
case "$r" in *COSIM_ACCEPT_ISAAC_EULA*) chk "EULA not accepted: message names COSIM_ACCEPT_ISAAC_EULA" ok;; *) chk "EULA not accepted: message names COSIM_ACCEPT_ISAAC_EULA" "$r";; esac

if [ "${COSIM_ACCEPT_ISAAC_EULA:-}" != Y ]; then
  chk "checks 4-6 need COSIM_ACCEPT_ISAAC_EULA=Y (NVIDIA Omniverse licence agreement, accepted by you)" "variable not set"
  echo "RESULT: FAIL"; exit 1
fi

r=$(py <<'E' 2>&1
import os, yaml, compose_gen
from unittest import mock
import simulators
cfg = yaml.safe_load(open(os.environ["CFG"]))
with mock.patch.object(simulators, "_docker_runtimes", return_value=["nvidia", "runc"], create=True):
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
if [ -f "$OUT/agent1_px4.log" ]; then
  grep -q "^PX4_GZ_" "$OUT/agent1_px4.log" && chk "no PX4_GZ_ variable in PX4's environment" "found" || chk "no PX4_GZ_ variable in PX4's environment" ok
fi

echo "RESULT: $([ $fail = 0 ] && echo PASS || echo FAIL)"
exit $fail
