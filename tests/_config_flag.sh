# Sourced (not run) by the test scripts, after ROOT is set: picks the experiment YAML.
#   --config <yaml> | --config=<yaml>   experiment file (default: config/experiments/two_agent_wifi.yaml)
# The CFG environment variable still works as a fallback. The flag may be given before or after the script name
# when run through tests/run_rootless.sh. Sets CFG to an absolute path. Leaves any other argument unconsumed
# and rejects it, since no test takes other arguments.
CFG="${CFG:-$ROOT/config/experiments/two_agent_wifi.yaml}"
while [ $# -gt 0 ]; do
  case "$1" in
    --config)   [ $# -ge 2 ] || { echo "--config needs a path"; exit 2; }; CFG="$2"; shift 2;;
    --config=*) CFG="${1#--config=}"; shift;;
    *) echo "unknown option: $1 (supported: --config <yaml>)"; exit 2;;
  esac
done
[ -f "$CFG" ] || { echo "config not found: $CFG"; exit 2; }
CFG="$(cd "$(dirname "$CFG")" && pwd)/$(basename "$CFG")"
