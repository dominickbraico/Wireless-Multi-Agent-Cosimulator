# Custom controllers

How to run your own ROS 2 packages on every agent, under the emulated radio, by editing only an experiment YAML. Your
nodes stay ordinary single-domain ROS 2 nodes. The YAML lists which topics cross the radio, and a relay in each agent
carries them.

## Contents

- [Quick start](#quick-start)
- [How it works](#how-it-works)
- [YAML keys](#yaml-keys)
- [Launch arguments](#launch-arguments)
- [What a node can see](#what-a-node-can-see)
- [Radio topics and QoS](#radio-topics-and-qos)
- [Run lifecycle](#run-lifecycle)
- [Outputs](#outputs)
- [Results](#results)
- [Limitations](#limitations)

## Quick start

Build the images for the template experiment. The first build compiles the template workspace into an overlay image:

```bash
docker build -t cosim-px4-runtime px4/
python3 orchestrator/cosim.py build --config config/experiments/custom_consensus.yaml
```

Fly the template controller, altitude consensus with four drones, with its pass and fail checks:

```bash
tests/containers_custom_template.sh
```

Start your own controller from the template:

1. Copy `examples/template_ws` and rename its packages. See
   [../examples/template_ws/README.md](../examples/template_ws/README.md).
2. Copy `config/experiments/custom_consensus.yaml` and point `scenario.custom` at your workspace and launch file.
3. List the topics your agents exchange under `scenario.custom.radio_topics`.
4. Rebuild with `python3 orchestrator/cosim.py build --config <your yaml>` after every change to your workspace.

## How it works

Each agent runs two DDS domains:

| Domain | Reach | Who uses it |
|---|---|---|
| 0, local | Inside the agent only | PX4 (`/fmu/...`) and your nodes |
| 42, radio | The emulated radio (`radio0`) | The relay only |

The relay (`ros2_ws/src/radio_relay`, C++) subscribes to every listed topic in each domain and republishes the
serialized bytes, unchanged, in the other domain, with the same name, type and QoS. It adds nothing to a message, so
the radio carries your messages as they are. Topics that are not listed never leave the agent.

```text
your nodes --(domain 0)--> relay --(domain 42, radio0)--> ns-3 802.11 channel --> relay of another agent --(domain 0)--> its nodes
```

The radio firewall and the isolation checks of the other scenarios apply unchanged. See
[ARCHITECTURE.md](ARCHITECTURE.md).

## YAML keys

`scenario.type: custom` selects the scenario. `orchestrator/custom_config.py` validates the section before anything
starts and names the offending key.

```yaml
scenario:
  type: custom
  custom:
    workspace: examples/template_ws
    launch:
      package: consensus_controller
      file: agent.launch.py
      args: {gain: 0.3}
    radio_topics:
      - {name: /swarm/state, type: consensus_msgs/msg/AgentState, reliability: best_effort, durability: volatile, depth: 10}
agents:
  - {name: agent1, ..., launch_args: {start_altitude: 4.0}}
```

| Key | Required | Meaning |
|---|---|---|
| `scenario.custom.workspace` | Yes | A colcon workspace with a `src/` directory; relative to the repository root, or absolute |
| `scenario.custom.launch.package` | Yes | The package in the workspace that holds the launch file |
| `scenario.custom.launch.file` | Yes | The launch file started once in every agent |
| `scenario.custom.launch.args` | No | Extra launch arguments, the same for every agent |
| `scenario.custom.radio_topics` | Yes | The topics that cross the radio; at least one |
| `radio_topics[].name` | Yes | Absolute topic name. Names under `/fmu` or `/px4_` are refused |
| `radio_topics[].type` | Yes | Message type, `package/msg/Name` |
| `radio_topics[].reliability` | No | `best_effort` (default) or `reliable` |
| `radio_topics[].durability` | No | `volatile` (default) or `transient_local` |
| `radio_topics[].depth` | No | Keep-last depth, integer >= 1. Default 10 |
| `agents[].launch_args` | No | Extra launch arguments for one agent; they override `launch.args` |
| `experiment.duration_s` | Yes | Longest run after the go signal; the launch is stopped then |

## Launch arguments

Every launch file receives these arguments. Declare all of them in your launch file, used or not, because `ros2
launch` refuses undeclared arguments.

| Argument | Value |
|---|---|
| `agent_name` | `agents[].name` |
| `agent_id` | `px4_instance` + 1; also PX4's MAVLink system id |
| `n_agents` | Number of agents in the YAML |
| `px4_instance` | `agents[].px4_instance` |
| `px4_ns` | Prefix of this vehicle's PX4 topics: empty for instance 0, `/px4_N` otherwise |
| `spawn_x`, `spawn_y`, `spawn_z`, `spawn_yaw` | `agents[].spawn_pose`: world frame (ENU), metres; yaw as given in the YAML |
| `log_dir` | `/logs/custom/<agent>`, a directory your nodes may write to |

`launch.args` follow, then `agents[].launch_args`. A later value replaces an earlier one with the same name.

An argument whose value is empty is not passed, because `ros2 launch` refuses empty values. Declare every argument
with a default; for `px4_ns` the default must be the empty string, which is the value for PX4 instance 0.

## What a node can see

| Visible | How |
|---|---|
| This vehicle's PX4 | `<px4_ns>/fmu/in/...` and `<px4_ns>/fmu/out/...` on domain 0. `px4_offboard_control.trajectory_base.TrajectoryNode` handles arming, take-off and landing |
| Other agents | Only through the topics in `radio_topics` |
| The world | Nothing directly. Positions of other agents arrive only if they send them over a radio topic |

PX4's local frame is NED, with its origin at the vehicle's arming point. The world frame of `spawn_pose` is ENU.

Your nodes receive their own publications on a radio topic, like any ROS 2 subscriber in the same domain. Filter them
by a sender id in the message, as the template does.

## Radio topics and QoS

The relay uses the QoS in the YAML on both sides. Your publishers and subscribers must use a compatible profile: a
best-effort publisher never matches a reliable subscriber, so a mismatch delivers nothing.

| Choice | Suits | Effect on the channel |
|---|---|---|
| `best_effort` | Periodic state where only the newest sample matters | Loss stays visible; no retransmission load |
| `reliable` | Messages other agents plan around | Loss becomes latency |
| `transient_local` | State that a late-joining agent needs | The relay resends its last samples to new subscribers |
| `depth` above 1 | A topic with several publishers | Simultaneous messages from different senders do not overwrite each other |

A message larger than one 1500-byte frame is fragmented. Under `best_effort`, losing one fragment loses the message.

## Run lifecycle

Each agent container runs these steps:

1. PX4 and the Micro XRCE-DDS Agent start; the agent reports PX4 ready.
2. The relay starts. If it exits within 3 s, for example on an unknown message type, the agent prints
   `relay failed: <reason>` and stops before the run begins.
3. The agent waits for the go signal, `/logs/go`, which the test driver creates once every agent is ready.
4. `ros2 launch <package> <file> <arguments>` starts on domain 0.
5. The run ends for this agent when the launch exits, or `experiment.duration_s` after the go signal, when the agent
   sends SIGINT and, 10 s later, SIGKILL.
6. The agent writes `/logs/<agent>.done`.

A launch that exits with an error within 5 s is reported as `launch failed, see <agent>_launch.log`.

The framework does not land the vehicles. Your controller lands them.

## Outputs

Files in the log directory, in addition to those of every run (see [EXPERIMENTS.md](EXPERIMENTS.md)):

| File | Contents |
|---|---|
| `<agent>_launch.log` | Output of your launch file |
| `custom/<agent>/` | Your nodes' `log_dir`, and `relay_topics.txt`, the topics the relay carries |
| `relay/relay_<agent>.csv` | One row per relayed message: `t_s,direction,topic,bytes,hash`; `tx` = sent to the radio, `rx` = received from it |
| `relay_<agent>.log` | The relay's log, including the QoS in effect per topic |
| `custom_report.json` | Written by `tools/analyze_custom.py` |

Compute delivery ratio and latency per topic and ordered agent pair from any run:

```bash
python3 tools/analyze_radio.py <log dir>
```

The analyzer matches sent and received messages by topic and payload hash, so your messages need no sequence numbers.
Latency covers relay to relay; the two loopback hops between your nodes and the relays are outside it.

## Results

The tests and their results are in [EXPERIMENTS.md](EXPERIMENTS.md), "Custom controller".

## Limitations

- Docker deployment only. The native deployment does not support this scenario.
- Tested with Python nodes only. C++ nodes are not tested.
- Not tested: Isaac Sim, rovers, more than four agents, more than one radio topic in a flight.
- The relay's throughput for large or fast messages, such as images, is not measured.
- The analyzer cannot tell apart identical payloads sent at the same time by two agents. Per-pair figures are then
  approximate.
- Every rebuild of your workspace rebuilds the overlay image. `rosdep` needs internet during that build.
- The relay and your package together would also be what runs on a robot. Running them on hardware is not tested.
