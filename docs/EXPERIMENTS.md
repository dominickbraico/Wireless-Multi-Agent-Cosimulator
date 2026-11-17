# Experiments and tests

The test scripts, their outputs, their pass criteria and the results measured with them. Every result names its date,
configuration and command. All results are single runs on one machine unless stated; they are not statistics.

## Contents

- [Usage](#usage)
- [Test scripts](#test-scripts)
- [Leader-follower flight](#leader-follower-flight)
- [Rover drive test](#rover-drive-test)
- [Isaac Sim stages](#isaac-sim-stages)
- [Demo with the Gazebo GUI](#demo-with-the-gazebo-gui)
- [Reference results](#reference-results)
- [Limitations](#limitations)

## Usage

Each script prints PASS and FAIL lines and takes `--config <yaml>`. The default configuration is
`config/experiments/two_agent_wifi.yaml` unless the table below names another.

Run a native script without sudo. The wrapper creates private user, network and mount namespaces and leaves the host
network untouched:

```bash
tests/run_rootless.sh tests/static_link.sh
```

Run a native script as root:

```bash
sudo tests/static_link.sh
```

Run a container script directly. It needs only Docker:

```bash
tests/containers_static_link.sh
```

## Test scripts

| Script | Deployment | Checks |
|---|---|---|
| `tests/static_link.sh` | Native | Ping and 2 Mbit/s UDP cross ns-3 with negligible loss; nothing flows with ns-3 stopped |
| `tests/ros2_isolation.sh` | Native | A ROS 2 talker and listener work through ns-3; no messages and no discovery with ns-3 stopped |
| `tests/swarm_qos.sh` | Loopback | `LeaderState` QoS: matching profiles deliver, mismatched profiles do not, invalid values are rejected |
| `tests/swarm_leader_follower.sh` | Native | The swarm node with a synthetic leader: loss, latency, tracking; nothing without ns-3 |
| `tests/pose_feed.sh` | Native | Positions applied in ns-3 match Gazebo within one update period |
| `tests/distance_sweep.sh` | Native | Moves one agent from 1 m to 200 m and back under traffic; plots RSSI, loss and latency |
| `tests/two_vehicles_fly.sh` | Native | Two PX4 vehicles each fly a square on the local domain |
| `tests/leader_follower_fly.sh` | Native | Leader-follower flight with every agent in the YAML |
| `tests/containers_static_link.sh` | Docker | Ping, UDP, isolation, pcap, lag, and that ns-3 runs the configured Wi-Fi standard |
| `tests/containers_leader_follower.sh` | Docker | The leader-follower flight with the same analysis and criteria |
| `tests/containers_rover.sh` | Docker | Two rovers each drive a square |
| `tests/mesh_plan.sh` | Offline | The mesh-dropout flight plans, both modes, 40 seeds each |
| `tests/containers_mesh_dropout.sh` | Docker | The mesh-dropout flight; criteria in [MESH_DROPOUT.md](MESH_DROPOUT.md) |
| `tests/px4_mavlink_link.sh` and four more | Docker | Isaac Sim stages A to F, below |

- `tests/static_link.sh` also checks the ns-3 lag and that the pcap holds 802.11 frames.
- `tests/distance_sweep.sh` takes about 3 minutes and compares against the prediction of `tools/link_budget.py`, which
  uses ns-3's table-based error model.
- All tests except the leader-follower flights and the mesh-dropout flight address `agent1` and `agent2` by name. A
  configuration used with them must contain those agents.

## Leader-follower flight

Fly every agent of the experiment, natively or in containers:

```bash
tests/run_rootless.sh tests/leader_follower_fly.sh --config config/experiments/five_agent_wifi.yaml
tests/containers_leader_follower.sh --config config/experiments/five_agent_wifi.yaml
```

The leader flies a square and publishes its position. Each follower receives it over the emulated link and flies to
`leader position + offset`. At the end, the leader's swarm node stops, and every follower must detect the link loss and
land.

| Environment variable | Meaning |
|---|---|
| `PX4_WORLD` | Gazebo world in `worlds/` |
| `SIDE`, `ALT`, `SPEED` | Square side (m), altitude (m), speed (m/s) |
| `OFFSET` | One offset forced on every follower |
| `OUT` | Log directory |

Outputs in the log directory:

| File | Contents |
|---|---|
| `formation_report.json` | All numbers, per follower |
| `formation.png` | Paths and formation error per follower |
| `comm_map.png` | Signal strength of every agent pair at mid-flight. See [SIGNAL_MODEL.md](SIGNAL_MODEL.md) |
| `link_vs_distance.png` | Swarm latency and loss against mean distance to the leader (more than one follower) |
| `swarm_follower_<agent>.csv` | Sequence number, latency and gap per message |
| `<agent>_flight.log`, `swarm_<agent>.log` | Node logs |
| `ns3/` | ns-3 summary, applied poses, `pair_rssi.csv`, lag and pcap files |
| `firewall_counters.txt` | What the radio firewall blocked, per agent |

Pass criteria:

| Criterion | Threshold |
|---|---|
| Flight | Every vehicle reaches altitude; the leader lands |
| Formation | Horizontal error p95 at most 5 m while the leader moves, against Gazebo ground truth |
| Stream | Sequence loss below 5 % per follower |
| Link loss | Every follower lands when the link goes away |
| Isolation | Only domain-42 traffic on the channel; no domain-0, XRCE or PX4-topic traffic |
| Validity | ns-3 lag within `time.max_lag_ms`; real-time factor at least `time.min_rtf` |

The formation threshold was fixed before the first run. "Run validity" in [ARCHITECTURE.md](ARCHITECTURE.md) defines
the validity window.

## Rover drive test

Start the Docker deployment with two Aion R1 rovers. The default configuration is
`config/experiments/rover_two_agent.yaml`:

```bash
tests/containers_rover.sh
```

Each rover arms in offboard mode, drives a 6 m square from its spawn point at 1 m/s and disarms. The two squares are
6 m apart. The test checks the vehicle in Gazebo, not the radio: no swarm traffic is sent, and ns-3 only receives the
poses.

| Item | Value |
|---|---|
| Vehicle | `agents[].vehicle: r1_rover`, PX4 airframe `4009_gz_r1_rover`, Gazebo model `r1_rover`, in the pinned PX4 commit |
| Scenario | `scenario.type: rover_drive`, driven by `orchestrator/scenario_rover.py` |
| Node | `rover_drive` in `ros2_ws/src/px4_offboard_control`; its docstring explains the choice of setpoint |
| Setpoint | PX4's generic offboard position setpoint, a target that moves at the set speed |
| Analyzer | `tools/analyze_rover.py` |

Pass criteria per rover, fixed before the first run:

| Criterion | Threshold |
|---|---|
| Sequence | Arms in offboard mode, reaches all four corners, disarms |
| PX4 position | Passes within 0.75 m of every corner |
| Ground truth | Passes within 1.0 m of every corner and ends within 1.5 m of the spawn point |
| Attitude | Ground-truth z varies by at most 0.35 m while driving |
| Validity | As for the leader-follower flight, measured from the go signal |

Outputs: `rover/rover_<agent>.csv` (PX4 local position), `<agent>_flight.log`, `rover_report.json`.

## Isaac Sim stages

The stages belong to the experimental Isaac Sim driver described in [CONTAINERS.md](CONTAINERS.md). The criteria of
each stage were fixed before its first run. Stages C to F need `COSIM_ACCEPT_ISAAC_EULA=Y` and images built with
`python3 orchestrator/cosim.py build --config config/experiments/isaac_one_agent.yaml`.

Hardware and versions of every result below, unless stated: Ubuntu 24.04, RTX 5060 Ti, driver 580.178.04, Isaac Sim
5.1.0, Pegasus Simulator v5.1.0, PX4 commit `2fc441493eb887199c279e2eff990f3f4cd0deef`.

### Stage A: PX4 attaches to an external MAVLink simulator

The test needs no GPU and no Isaac Sim. It starts a stub TCP server (`tools/mavlink_sim_stub.py`) and PX4 from the
`cosim-px4-runtime` image with `PX4_SIM_MODEL=none_iris` and `PX4_SIM_HOST_ADDR` set to the stub:

```bash
tests/px4_mavlink_link.sh [--instance N]
```

| Criterion | Threshold |
|---|---|
| Connection | The PX4 log contains `Simulator connected on TCP port <4560+N>.` |
| Traffic | The stub receives at least 1 byte from PX4 |
| Environment | The PX4 environment has no `PX4_GZ_*` variable |

| Date | Run | Outcome |
|---|---|---|
| 2026-11-03 | Instance 0 (port 4560), single run | PASS |
| 2026-11-03 | Instance 1 (port 4561), single run | PASS |

The stage shows the connection only. The stub sends no sensor data and does not parse MAVLink.

### Stage B: Isaac Sim, Pegasus and the pinned PX4, outside the project's stack

Criterion: one Iris is armed and taken off by command. Within 60 s of the take-off command, both PX4's local position
and the vehicle's height in Isaac Sim rise by at least 2.0 m.

| Date | Run | Outcome |
|---|---|---|
| 2026-11-03 | Single run, headless, container `nvcr.io/nvidia/isaac-sim:5.1.0` | PASS |

Measured in that run:

| Quantity | Value |
|---|---|
| Height in Isaac Sim | From 0.07 m to 2.0 m within 10 s; settled at about 2.58 m |
| PX4 local z | -2.52 m (NED) |
| Simulated time over wall time, 10 s windows, whole run | Minimum 0.55, median 0.85, maximum 1.31 (n = 193) |
| PX4 `poll timeout` errors during start-up | 32 |

Conditions: Pegasus listened on TCP 4560 with `connection_ip` set to `0.0.0.0` and `px4_autolaunch` false. PX4 ran in a
second container and was commanded with `px4-commander`. The scene was Isaac's default ground plane, loaded from
NVIDIA's asset server; this run's container had internet access. Pegasus documents PX4 1.14.3 as its pairing; the
pinned PX4 is `v1.18.0-beta1-706-g2fc441493e`.

Not tested in this stage: flight beyond take-off, several vehicles, ns-3, a rendered scene, sensors.

### Stage C: the `isaac` simulator driver

Run the driver checks with `config/experiments/isaac_one_agent.yaml`:

```bash
COSIM_ACCEPT_ISAAC_EULA=Y tests/containers_isaac_ready.sh
```

| Check | Criterion |
|---|---|
| 1 | A vehicle with no Isaac definition (`r1_rover`) is refused by name |
| 2 | Without an `nvidia` runtime, the generator stops with a message that names the NVIDIA Container Toolkit |
| 3 | Without `COSIM_ACCEPT_ISAAC_EULA=Y`, it stops with a message that names the variable |
| 4 | The Compose file reserves a GPU for the simulator service only, and that service has no `GZ_` variable |
| 5 | `cosim up` brings the simulator, ns-3 and agent1 up healthy, and agent1 reports PX4 ready |
| 6a, 6b | The running `px4` process has `PX4_SIM_MODEL=none_iris` and no `PX4_GZ_*` variable |
| 7 | A `camera` block without a positive `width`, `height` or `rate_hz` is refused with a message that names the key |

| Date | Criterion change | Reason |
|---|---|---|
| 2026-11-04 | Check 6 reads the environment of the running `px4` process from `/proc` (6a, 6b) | As first written it read PX4's log, which never prints the environment, so it could not fail |
| 2026-11-04 | Check 7 added | Review of the camera configuration |

| Date | Run | Outcome |
|---|---|---|
| 2026-11-03 | Single run, checks 1 to 6 in their first form | All PASS; check 6 rested on the form that could not fail |
| 2026-11-04 | Single run, checks 1 to 7 | All PASS |

The stage shows start-up only. The Gazebo path was re-run after the change: `tests/containers_leader_follower.sh`
(default configuration, single run) passed all checks, with formation error p95 1.79 m, swarm loss 0 % and a worst
real-time-factor window of 1.000.

### Stage D: pose feed to ns-3

Run the pose-feed checks with `config/experiments/isaac_one_agent.yaml`:

```bash
COSIM_ACCEPT_ISAAC_EULA=Y tests/isaac_pose_feed.sh
```

Part A tests `adapters/isaac_sim/pose_feed.py` against the pose feed contract without Docker. Part B runs the stack,
arms agent1's PX4 and sends `takeoff`.

| Check | Criterion |
|---|---|
| A1 | Datagrams are JSON with exactly the keys `seq`, `t_sim`, `t_wall`, `agents`; positions equal the stored position plus `antenna_offset` |
| A2 | The rate is the configured one within 30 %; `seq` rises by 1; `t_wall` does not decrease |
| A3 | After 1.5 s without an update, the last pose is still sent at the same rate and `t_wall` advances |
| A4 | A new position is sent after the next update |
| B1 | The antenna z in `ns3/pose_applied.csv` rises by `MIS_TAKEOFF_ALT` within 0.5 m |
| B2 | No gap longer than 1.0 s between applied samples after arming |

| Date | Run | Outcome |
|---|---|---|
| 2026-11-03 | First full run | B1 FAIL from a test defect: the test read `MIS_TAKEOFF_ALT` with `param show`, which prints nothing for an unused parameter. The criterion was not changed. |
| 2026-11-03 | Single run after the fix (`param show -q`) | All PASS. B1: rise 2.52 m for 2.50 m, resting antenna z 0.16 m. B2: longest gap 0.02 s over 2272 samples |

The stage shows that the vehicle position in Isaac Sim reaches ns-3 and follows a take-off.

### Stage E: real-time factor of an Isaac Sim run

Isaac Sim has no equivalent of Gazebo's `/stats` topic. `rtf_from_poses` in `tools/timehealth.py` takes the factor from
the pose feed: simulated seconds (`t_sim`) over wall seconds (`t_wall_apply_s`) across 5 consecutive applied samples.
The feed repeats the last pose while the simulator is blocked, so a stall counts as a factor of 0.

`tools/analyze_isaac.py <log dir> <yaml>` judges a run. The criterion is the `time.rtf_window_s` sliding mean against
`time.min_rtf` over the flight window. The flight window runs from the first sample whose antenna z is 0.3 m above the
median z of the first 10 s, to the last sample. A run with no pose samples or no flight is "run not certified".

Test the function on synthetic streams and the analyzer on a run directory:

```bash
tests/timehealth_isaac.sh [--logs DIR]
```

| Synthetic stream | Expected |
|---|---|
| Real time | Factor 1.0 |
| Half speed | Factor 0.5 |
| A 2 s stall | Factor 0 in the stall; the sliding mean fails |
| Too few samples | No value |

The criteria test that a number is produced correctly, not that Isaac Sim meets `min_rtf`.

| Date | Run | Outcome |
|---|---|---|
| 2026-11-03 | `tests/timehealth_isaac.sh --logs <stage D run>`, single run | All PASS |
| 2026-11-03 | The stage D run: one Iris, empty scene, no sensors, take-off to 2.5 m | Flight window (40.3 s): worst 5 s mean 0.975, worst sample 0.68. Whole stream: worst 5 s mean 0.731. ns-3 lag at most 1.687 ms |
| 2026-11-03 | Cross-check on a Gazebo run of `tests/containers_leader_follower.sh` | Pose-derived: median 1.000, worst 5 s mean 0.997, samples 0.80 to 1.20. Gazebo `/stats`: median 1.000, worst 5 s mean 1.000 |

- The pose-derived value agrees with Gazebo's own on the sliding mean and is noisier per sample.
- The stage B figures were measured over a whole run that includes the wait for PX4. They are not comparable with the
  flight-window figure.
- The flight-window mean of the stage E run was above 1. A later version of the analyzer printed 1.158 for that log.
  The log directory was not kept, so that number is not reproducible.

### Stage F: camera path

One Isaac Sim camera on the vehicle (`agents[].camera`) streams over the management network to the agent container,
never over the emulated radio. Run it with `config/experiments/isaac_camera.yaml` (320 x 240 at 15 Hz):

```bash
COSIM_ACCEPT_ISAAC_EULA=Y tests/isaac_camera_path.sh
```

| Check | Criterion |
|---|---|
| Part A | The frame protocol and the receiver `orchestrator/camera_sink.py` work without Docker (A1 to A6) |
| F1 | The average frame rate at the receiver, from arming to the end of the flight window, is at least 10 frames/s |
| F2 | Each frame has `width x height x 3` bytes, and at least 90 % of frames are not constant |
| F3a | The receiver's recorded sender address equals the simulator's management address |
| F3b | The ns-3 pcap holds frames, and none to or from TCP port 5700 + `px4_instance` |
| F4 | The flight-window real-time factor stays at or above `time.min_rtf` (0.95) with the camera on |

A frame is not constant when the standard deviation of a byte sample exceeds 1.0.

| Date | Criterion change | Reason |
|---|---|---|
| 2026-11-04 | F3a added; the former F3 becomes F3b | The pcap check alone is weak: the stream is on the management network by construction and the radio was nearly idle |
| 2026-11-04 | Part A checks A5 and A6 added | The receiver now binds the agent's management address only and keeps partial frames across receive timeouts |

| Date | Run | F1 (frames/s) | F2 | F3 | F4 worst 5 s mean | Flight-window mean | Outcome |
|---|---|---|---|---|---|---|---|
| 2026-11-03 | 1 | 16.9 (766 frames in 45.2 s) | Pass | Pass, 9 frames in the pcap | 0.986 | 1.12 | All PASS |
| 2026-11-03 | 2, with `save_every: 100` | 15.6 (706 frames in 45.2 s) | Pass | Pass | 0.952 | 1.03 | All PASS |
| 2026-11-04 | 3, after the review fixes | 16.2 | Pass | F3a and F3b pass | 0.900 | 1.062 | F4 FAIL |
| 2026-11-04 | 4, after the review fixes | Not recorded here | Pass | F3a and F3b pass | 1.021 | 1.111 | All PASS |

Observations:

- Each frame is 230 400 bytes, so the stream is about 3.6 MB/s. All frames were 320 x 240 x 3 bytes and none was
  constant.
- Frames rendered before the receiver connects are dropped. In run 1, sequence numbers 1 to 74 were missing.
- Saved frames of run 2 show the lit scene: a red post, grey ground, horizon and sky.
- The offline simulator container logs material compile errors for materials on NVIDIA's asset server (`Linen_Blue`,
  `Carpet_Gray`). The saved frames show no sign of them.
- The highest 5 s window mean was 1.219 in run 3 and 1.236 in run 4.
- F4 is met in three of four runs. The margin is small, and the run-to-run variation exceeds any difference between
  runs with and without the camera (stage E, no camera, one run: 0.975). The cause of the 0.900 run is not
  investigated. The threshold is unchanged.

Finding: Isaac Sim's simulation is not paced to wall time. A frame rate above the configured 15 Hz follows from
simulated time running faster than wall time. Gazebo holds its factor at 1.000 by its own setting. A factor above 1
means that vehicle dynamics advance faster than the wall-clock ROS 2 nodes and ns-3 expect, and the validity check,
a lower bound, does not catch it. `tools/analyze_isaac.py` prints the flight-window mean and the highest 5 s window
mean, and a `WARN  not paced to wall time` line when the flight-window mean exceeds 1.05. The value 1.05 was chosen on
2026-11-04 without reference to a result. It is a warning, not a criterion.

### Four multirotors in the mesh-dropout scenario

Run the mesh-dropout scenario with four Pegasus Iris vehicles in Isaac Sim. The configuration uses the scenario and
radio settings of `four_agent_mesh_dropout.yaml` (seed 7, transmit power 4 dBm, *D* = 20.5 m):

```bash
COSIM_ACCEPT_ISAAC_EULA=Y tests/containers_mesh_dropout.sh --config config/experiments/isaac_four_agent_mesh_dropout.yaml
```

The criteria are those of [MESH_DROPOUT.md](MESH_DROPOUT.md), fixed before the first run on 2026-11-10, with one
substitution. The real-time factor comes from the pose feed of the first agent (`rtf_from_poses`), judged over the same
window against the same `time.min_rtf` (0.95) with the same 5 s sliding mean. A run without a real-time-factor source
fails that check.

| Date | Run | Outcome |
|---|---|---|
| 2026-11-10 | Single run, headless, no camera, 16 cores | Not valid: the real-time-factor check FAILED. Every other check passed. |

| Quantity | Value |
|---|---|
| Real-time factor, worst 5 s window mean | 0.291 |
| Real-time factor, flight-window mean; highest 5 s window mean | 0.594; 0.666 |
| Real-time factor, 10 s means over the whole run | 0.26 to 0.32 for the first 40 s of the pose stream, then 0.58 to 0.65 |
| Distance from spawn after touchdown | 0.03 m to 0.21 m |
| `LOST` after the modelled drop; `UP` relative to the modelled recovery | 1.0 s to 1.3 s; -0.4 s to +0.7 s |
| agent3 fully disconnected | 17.8 s |
| ns-3 lag | At most 7.87 ms (limit 10 ms) |

- All four vehicles armed, took off, flew their plans in offboard mode, landed and disarmed. ns-3 modelled the three
  planned drops of agent3's links, and the heartbeats showed them in both directions. No `LOST` event was unexplained.
- Simulated time ran at about 0.6 of wall time while ns-3 and the ROS 2 nodes ran at wall time. The figures show that
  the pipeline works with several Isaac vehicles. They are not valid measurements of the scenario.
- One Iris runs at or above 1.0 (stages E and F), so four vehicles do not hold real time on this machine with the scene
  script as written.
- A likely cause, read from the code and not measured: with Pegasus' default lockstep (`enable_lockstep`), each 250 Hz
  physics step waits in turn for an actuator message from each vehicle's PX4, so the waits add up.
- This is the first run with more than one Isaac vehicle, and the first Isaac flight commanded by the ROS 2 offboard
  nodes. Stages B and D use `px4-commander`.

## Demo with the Gazebo GUI

Run the leader-follower flight natively with the Gazebo GUI. The default world is Baylands, whose terrain downloads
from Gazebo Fuel on first use (about 500 MB). The demo needs sudo:

```bash
demo/run_gui_demo.sh [--config YAML] [--world NAME] [--record] [--wireshark] [--no-gui] [--no-prompt]
```

| Tool | Purpose |
|---|---|
| `demo/monitor.py <log dir>` | Live terminal dashboard |
| `demo/cleanup.sh [--config YAML]` | Stops leftover processes and removes network plumbing after an interrupted run |
| `demo/run_container_demo.sh` | The same demo on the Docker deployment, without sudo, with a live signal map. See [CONTAINERS.md](CONTAINERS.md) |

The demo flies every agent in the configuration.

With an Isaac Sim configuration, `demo/run_container_demo.sh` opens Isaac Sim's own window instead of the Gazebo GUI:

```bash
COSIM_ACCEPT_ISAAC_EULA=Y demo/run_container_demo.sh --config config/experiments/isaac_four_agent_mesh_dropout.yaml
```

The script sets `COSIM_ISAAC_GUI=1`. The simulator container then runs Isaac Sim with a window on the host's X11
display; the X11 socket and a copy of the display's cookie are mounted into the container, and its network stays
internal. The scene is the flat ground box with two lights. The window redraws 20 times per wall-clock second.
`--world` and the trail markers do not apply. The host needs an X11 session and `xauth`.

| Date | Run | Outcome |
|---|---|---|
| 2026-11-10 | Single run, `--no-prompt`, same hardware and versions as the four-multirotor run | The window opened (1440 x 900) and the live map ran. All flight, link and isolation checks passed. The real-time-factor check FAILED: worst 5 s window mean 0.465, flight-window mean 0.547 |

A GUI run is a demonstration, not a valid measurement.

## Reference results

Single runs on the development machine (16 cores, 31 GB), Cyclone DDS, world `px4_flat`, a 10 m square at 5 m altitude
and 1.5 m/s. The native runs use `tests/run_rootless.sh tests/leader_follower_fly.sh`; the Docker runs use
`tests/containers_leader_follower.sh`. They show that the pipeline works.

| Configuration | Deployment | Followers | Formation error p50 / p95 (m) | Swarm loss | Swarm latency p50 / p99 (ms) | ns-3 max lag (ms) |
|---|---|---|---|---|---|---|
| `five_agent_wifi.yaml` | Native | 4, at 8 m to 32 m | 1.62 to 1.65 / 1.74 to 1.79 | 0 % | 1.15 to 1.28 / 1.74 to 2.34 | 0.11 |
| `two_agent_wifi.yaml` | Native | 1, at 3 m | 1.64 / 1.82 | 0 % | 1.06 / 1.44 | 0.09 |
| `five_agent_wifi.yaml` | Docker | 4, at 8 m to 32 m | 1.65 to 1.68 / 1.81 to 1.88 | 0 % | 1.37 to 1.56 / 2.10 to 2.67 | 0.57 |
| `two_agent_wifi.yaml` | Docker | 1, at 3 m | 1.67 / 1.84 | 0 % | 1.11 / 1.57 | 0.28 |

- The steady formation error of about 1.7 m equals roughly 1.1 s of lag at 1.5 m/s. The radio is not the cause.
- With the default receiver model, all followers are inside the range of about 51 m, so latency changes little with
  distance. Gradual degradation needs the `per_limited` receiver model with followers near 100 m.

Distance at which UDP loss reaches 1 %, 50 % and 99 % in the distance sweep:

| Configuration | Distance at 1 % / 50 % / 99 % loss |
|---|---|
| `two_agent_wifi.yaml` | 51.5 m, a hard step |
| `two_agent_wifi_snr_floor.yaml` | 95 m, a hard step |
| `two_agent_wifi_per_limited.yaml` | 101 m / 105.5 m / 110.5 m |

## Limitations

- Runs are real-time. A run is valid only if the lag and real-time-factor checks pass. A machine with too few cores for
  the number of vehicles fails them.
- Results are single runs. Repeat them and report variance before citing them.
- The distance-sweep figures come from runs that predate the current script layout. They are not repeated.
- The cause of the 1.7 m formation error is not tested. It is consistent with PX4's position-loop response combined
  with position-only setpoints (no velocity feed-forward).
- Extra gz-transport clients during a measured window, for example one `gz service` call per measurement, stall the
  Gazebo server and lower the real-time factor.
- The demo's overview camera and `demo/monitor.py` are laid out for the two-agent configuration. The dashboard shows
  only `agent1` and `agent2`.
- Isaac Sim: the agreement of its world frame with ns-3's frame is assumed, not tested. Pacing the simulation to wall
  time and an upper bound on the real-time factor are not implemented.
- Isaac Sim stage C does not fly the vehicle; stage D does not measure the real-time factor; stage E uses an empty
  scene with no sensors.
- Isaac Sim stage F: material realism offline is not established, and a camera penalty on the real-time factor cannot
  be excluded.
- Isaac Sim with several vehicles does not hold real time on the development machine, and the cause is not
  investigated. Not tested: two or three vehicles, lockstep off, a camera with several vehicles.
- The rendered picture of the Isaac Sim window (lighting, camera framing) is not inspected.
