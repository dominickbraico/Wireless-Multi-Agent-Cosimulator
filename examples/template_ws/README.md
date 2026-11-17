# Template workspace

A minimal ROS 2 workspace that runs under the cosimulator as a custom controller. Copy it to start your own. The
contract it follows is in [../../docs/CUSTOM_CONTROLLERS.md](../../docs/CUSTOM_CONTROLLERS.md).

## Contents

| Path | Purpose |
|---|---|
| `src/consensus_msgs/` | Message package (`ament_cmake`): `AgentState.msg`, the one message sent over the radio |
| `src/consensus_controller/consensus_controller/consensus_node.py` | The controller node: altitude consensus |
| `src/consensus_controller/launch/agent.launch.py` | The launch file the cosimulator starts once per agent |

The matching experiment file is `config/experiments/custom_consensus.yaml`.

## Behaviour

Each drone takes off to its own `start_altitude`. At `rate_hz` it broadcasts its altitude setpoint on `/swarm/state`
and moves the setpoint toward the mean of the values it heard in the last second:

```text
x_i  <-  x_i + gain * (mean of x_j heard in the last 1.0 s - x_i)
```

The drones hold their horizontal position, so they cannot collide. After `run_s` seconds each drone lands. With the
radio in range, the altitudes converge to a common value. Without it, each drone keeps its own.

| Launch argument | Default | Meaning |
|---|---|---|
| `start_altitude` | 5.0 | Altitude after take-off, metres above the arming point |
| `gain` | 0.3 | Consensus gain per update |
| `run_s` | 40.0 | Duration of the consensus phase, seconds |
| `rate_hz` | 5.0 | Broadcast and update rate |
| `dry_run` | false | Runs the consensus exchange without a vehicle; used by `tests/custom_relay.sh` |

## Usage

Build the images, then fly the template with its pass and fail checks:

```bash
python3 orchestrator/cosim.py build --config config/experiments/custom_consensus.yaml
tests/containers_custom_template.sh
```

Check the controller without vehicles or Docker. This needs ROS 2 Jazzy on the host:

```bash
(cd ros2_ws && colcon build --symlink-install --packages-select radio_relay)
(cd examples/template_ws && colcon build)
source ros2_ws/install/setup.bash
source examples/template_ws/install/setup.bash
tests/custom_relay.sh
```

## Making your own controller

1. Copy `examples/template_ws` to a directory of your choice and rename the packages.
2. Copy `config/experiments/custom_consensus.yaml` and set `scenario.custom.workspace`, `launch.package` and
   `launch.file`.
3. List every topic that agents exchange under `scenario.custom.radio_topics`, with the QoS your nodes use.
4. Keep every framework argument declared in your launch file. `agent.launch.py` shows the list.
5. Rebuild the images with `cosim.py build --config <your yaml>` after each change to your workspace.

## Limitations

- The template is a Python node. A C++ node is not tested.
- The consensus state is the altitude setpoint, not the measured altitude.
- The node relies on the host clock to age out old values. Agents in the cosimulator share one clock; real vehicles do
  not.
