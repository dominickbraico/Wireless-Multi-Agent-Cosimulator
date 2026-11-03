# Experiments and tests

Each script prints PASS/FAIL lines. All take `--config <yaml>` (default `config/experiments/two_agent_wifi.yaml`).
Run a script without sudo through `tests/run_rootless.sh <script>` (private user, network and mount namespaces, the host
network is not touched), or as root with `sudo <script>`.

| Script | Purpose |
|---|---|
| `tests/static_link.sh` | Ping and 2 Mbit/s UDP across ns-3 with negligible loss, lag within limit, 802.11 frames in the pcap, nothing flows with ns-3 stopped |
| `tests/ros2_isolation.sh` | A ROS 2 talker and listener work through ns-3; zero messages and zero discovery with ns-3 stopped |
| `tests/swarm_qos.sh` | LeaderState QoS parameters: matching profiles deliver, a best-effort publisher does not match a reliable subscriber, invalid values are rejected. Loopback only, no ns-3 |
| `tests/swarm_leader_follower.sh` | Swarm node with a synthetic leader over the link: loss, latency, tracking, nothing without ns-3 |
| `tests/pose_feed.sh` | Positions applied in ns-3 match Gazebo within one update period |
| `tests/distance_sweep.sh` | About 3 minutes. Moves one agent from 1 to 200 m and back under traffic; plots RSSI, loss and latency against the link-budget prediction |
| `tests/two_vehicles_fly.sh` | Two PX4 vehicles each fly a square on the local domain |
| `tests/leader_follower_fly.sh` | Leader-follower flight with N vehicles (below) |
| `tests/containers_static_link.sh` | Container version of `static_link.sh`: ping, UDP, isolation (management and link networks, ns-3 stopped), pcap, lag, and that ns-3 ran the configured Wi-Fi standard. Only Docker needed |
| `tests/mesh_plan.sh` | Offline checks of the mesh-dropout flight plans, both modes, 40 seeds each: repeatable and different per seed; roam: an independent re-simulation confirms the at-risk limit, the clear drops, the full isolation of one agent (`min_isolation_s`), paths from and to the spawn point at speed inside the allowed area; excursion: exactly *k* links dropped with the margin; bad YAML rejected. Seconds, no Docker ([MESH_DROPOUT.md](MESH_DROPOUT.md)) |
| `tests/containers_mesh_dropout.sh` | Mesh-dropout flight in containers (default `four_agent_mesh_dropout.yaml`; also `..._baylands.yaml`): every agent flies a seeded random path (mode `roam`, the default) or one wanderer makes an excursion (mode `excursion`); links drop and recover; checked in the model (`pair_rssi.csv`) and in the agents' heartbeats; criteria in [MESH_DROPOUT.md](MESH_DROPOUT.md). Env `MESH_SEED` |
| `tests/containers_leader_follower.sh` | Container version of `leader_follower_fly.sh`: same flight, analysis and pass criteria, one container per agent. Only Docker needed ([CONTAINERS.md](CONTAINERS.md)) |

All tests except `leader_follower_fly.sh` address `agent1` and `agent2` by name; a config used with them must contain those agents. `tools/link_budget.py` predicts the range from the YAML (it uses ns-3's table-based error model).

## Leader-follower flight

`tests/leader_follower_fly.sh [--config YAML]` flies every agent of the experiment:
the leader flies a square and publishes its position; each follower receives it over the emulated link and flies to
`leader position + its offset` (see [CONFIGURATION.md](CONFIGURATION.md)). At the end the leader's swarm node is stopped and
every follower must detect the link loss and land. Environment: `PX4_WORLD` (Gazebo world in `worlds/`), `SIDE` (square side, m),
`ALT` (altitude, m), `SPEED` (m/s), `OFFSET` (force one offset), `OUT` (log directory).

Outputs in the log directory:

| File | Contents |
|---|---|
| `formation_report.json` | All numbers, per follower |
| `formation.png` | Paths and per-follower formation error |
| `comm_map.png` | 2D map of every agent pair's signal strength at mid-flight, dark green (strong) to red (weak) ([tools/plot_comm_map.py](../tools/plot_comm_map.py)); the values come from `ns3/pair_rssi.csv`; pairs below the receiver floor have no line, and the transmit power and the distance at which the signal reaches the floor are printed on the figure; derivation in [SIGNAL_MODEL.md](SIGNAL_MODEL.md) |
| `ns3/pair_rssi.csv` | Signal strength of every ordered agent pair every `network.link_log_interval_s`: `t_s,tx,rx,distance_m,rssi_dbm,below_floor`. Evaluated by ns-3 on its channel loss chain at the current node positions, so it exists for pairs that never talk |
| `link_vs_distance.png` | Swarm latency and loss against mean distance to the leader (more than one follower) |
| `swarm_follower_<agent>.csv` | Per-message sequence number, latency and gap for that follower |
| `<agent>_flight.log`, `swarm_<agent>.log` | Node logs |
| `ns3/` | ns-3 summary, applied poses, RSSI, lag and pcap files |
| `firewall_counters.txt` | What the radio firewall blocked, per agent |

Pass criteria: every vehicle reaches altitude; the leader lands; each follower holds its formation (horizontal error p95 at most
5 m while the leader moves, a threshold fixed before the first run), receives the stream with less than 5 % sequence loss and
lands when the link goes away; no domain-0, XRCE or PX4-topic traffic and only domain-42 traffic on the channel; ns-3 lag within
`max_lag_ms`; Gazebo real-time factor within `min_rtf` (see ARCHITECTURE.md, "Run validity", for the window). Formation error is measured against Gazebo ground truth.

## Rover drive test

`tests/containers_rover.sh [--config YAML]` (default `config/experiments/rover_two_agent.yaml`; images first: `python3 orchestrator/cosim.py build`)
starts the Docker deployment with two Aion R1 rovers (`agents[].vehicle: r1_rover`; PX4 airframe `4009_gz_r1_rover`, Gazebo model `r1_rover`, both in the
PX4 commit pinned in `px4/Dockerfile`). Each rover arms in offboard mode, drives a 6 m square from its spawn point at 1 m/s using PX4's generic offboard position setpoint
(a moving target at the set speed; why not the rover-specific setpoint: the docstring of `rover_drive.py`), and disarms (`scenario.type: rover_drive`; `orchestrator/scenario_rover.py`, `ros2_ws/.../rover_drive.py`). The two squares are 6 m apart. It checks the vehicle in Gazebo,
not the radio: no swarm traffic is sent, and ns-3 only receives the poses.

Pass criteria (thresholds fixed before the first run; `tools/analyze_rover.py`), per rover:

- it armed in offboard mode, reached all four corners and disarmed;
- PX4's own position passed within 0.75 m of every corner;
- the Gazebo ground truth (poses applied in ns-3) passed within 1.0 m of every corner and ended within 1.5 m of the spawn point;
- ground-truth z varied by at most 0.35 m while driving (not flipped or airborne).

Run validity as for the leader-follower flight: ns-3 lag within `max_lag_ms`; Gazebo real-time factor within `min_rtf`, measured from the go signal.
Outputs in the log directory: `rover/rover_<agent>.csv` (PX4 local position), `<agent>_flight.log`, `rover_report.json`.

## Isaac Sim stage A: PX4 attaches to an external MAVLink simulator

Part of the Isaac Sim spike (not implemented as a supported simulator; see [CONTAINERS.md](CONTAINERS.md)). It tests only PX4's side of the
link, with no GPU and no Isaac Sim: `tests/px4_mavlink_link.sh [--instance N]` starts a stub TCP server (`tools/mavlink_sim_stub.py`) on a fresh
Docker network and PX4 from the `cosim-px4-runtime` image (PX4 `2fc441493eb887199c279e2eff990f3f4cd0deef`) with `PX4_SIM_MODEL=none_iris` and
`PX4_SIM_HOST_ADDR` set to the stub. PX4 uses TCP port 4560 + instance (`px4-rc.mavlinksim` in the image).

Criteria, fixed before the first run: the PX4 log contains `Simulator connected on TCP port <4560+N>.`; the stub receives at least 1 byte from PX4;
the PX4 environment has no `PX4_GZ_*` variable.

Result (2026-11-03, single run per instance, image `sha256:555d8641e877…`): instance 0 (port 4560) and instance 1 (port 4561) both PASS.
This shows the connection only. The stub sends no sensor data and does not parse MAVLink, so it says nothing about flight, about Pegasus or Isaac Sim
compatibility, or about timing.

## Isaac Sim stage B: Isaac Sim + Pegasus + the pinned PX4, outside the project's stack

Part of the Isaac Sim spike (see above). Criterion, fixed on 2026-11-03 before the first stage B run: with Isaac Sim 5.1.0 (container `nvcr.io/nvidia/isaac-sim:5.1.0`),
Pegasus Simulator v5.1.0 and PX4 from the `cosim-px4-runtime` image attached over the simulator MAVLink interface, one Iris is armed and taken off by command, and
within 60 s of the takeoff command both PX4's own local position and the vehicle's height in Isaac Sim have risen by at least 2.0 m above the start. The result is recorded
below with the exact commands once it exists. If it fails, the failure is reported, not worked around.

Result (2026-11-03, single run, RTX 5060 Ti, driver 580.178.04, Ubuntu 24.04 host): PASS. Isaac Sim 5.1.0 headless with Pegasus Simulator v5.1.0
(commit `644da37e9d5268e5f9a34e78bdcfd57a8bab82b4`, used unmodified from `PYTHONPATH`; its `pip install -e` was not used because its setup edits Isaac Sim's kit app files) listened on TCP 4560
(`connection_ip` set to `0.0.0.0`, `px4_autolaunch` false, Isaac's default ground plane, which is loaded from NVIDIA's asset server: this run's container had internet access; corrected 2026-11-03, an earlier version of this text called it procedural). PX4 `v1.18.0-beta1-706-g2fc441493e` (the pinned commit; Pegasus' documented
pairing is PX4 1.14.3) ran in a second container with `PX4_SIM_MODEL=none_iris` and `PX4_SIM_HOST_ADDR` set to the Isaac container, and logged `Simulator connected on TCP port 4560.` and
`Ready for takeoff!`. After `px4-commander --instance 0 arm` and `takeoff`, the vehicle's height in Isaac went from 0.07 m to 2.0 m within 10 s and settled at about 2.58 m (PX4's default takeoff altitude);
PX4 reported local z = -2.52 m (NED). Isaac's simulated time over wall time, in 10-s windows over the whole run (including the wait for PX4), was
min 0.55, median 0.85, max 1.31 (n = 193): this is one run with an empty scene and no sensors, and it shows the real-time factor is not steady at 1.0. PX4 logged 32 `poll timeout` errors
during start-up while it waited for the first sensor data. Not tested: flight beyond takeoff, several vehicles, ns-3, a rendered scene, sensors.

## Isaac Sim stage C: the `isaac` simulator driver

Experimental (see [CONTAINERS.md](CONTAINERS.md), "Isaac Sim (spike, experimental)"). `COSIM_ACCEPT_ISAAC_EULA=Y tests/containers_isaac_ready.sh` with `config/experiments/isaac_one_agent.yaml`
(images first: `python3 orchestrator/cosim.py build --config config/experiments/isaac_one_agent.yaml`). Criteria, fixed before the first run on 2026-11-03: (1) a vehicle with no Isaac definition (`r1_rover`)
is refused with its name; (2) without an `nvidia` runtime the generator stops with a message naming the NVIDIA Container Toolkit; (3) without `COSIM_ACCEPT_ISAAC_EULA=Y` it stops with a message naming the variable;
(4) the generated Compose file reserves a GPU for the simulator service and for no other service, and the simulator service has no `GZ_` variable; (5) `cosim up` brings the stack up (simulator, ns-3 and
agent1 healthy) and agent1 reports PX4 ready; (6) PX4's start-up log shows no `PX4_GZ_` variable.

Result (2026-11-03, single run, Isaac Sim 5.1.0 + Pegasus v5.1.0, RTX 5060 Ti, driver 580.178.04, image `cosim-sim-isaac` built from `nvcr.io/nvidia/isaac-sim:5.1.0`): all six checks PASS.
Two earlier runs failed and led to fixes recorded in [CONTAINERS.md](CONTAINERS.md): the simulator container's offline network made Pegasus' import and Isaac's default ground plane fail. In the passing run agent1's PX4 logged
`Simulator connected on TCP port 4560.` to the simulator at the management address and `Ready for takeoff!`. This shows start-up only: the vehicle was not flown, no pose reached ns-3, and the real-time factor was not measured.
The Gazebo path was re-run after the change: `tests/containers_leader_follower.sh` (default config, single run) passed all checks (p95 formation error 1.79 m, 0 % swarm loss, RTF worst window 1.000).

## Isaac Sim stage D: pose feed to ns-3

Experimental (see [CONTAINERS.md](CONTAINERS.md), "Isaac Sim (spike, experimental)"). `COSIM_ACCEPT_ISAAC_EULA=Y tests/isaac_pose_feed.sh` with `config/experiments/isaac_one_agent.yaml`. Part A, without Docker, tests
`adapters/isaac_sim/pose_feed.py` against the pose-feed contract: (A1) datagrams are JSON with exactly the keys `seq`, `t_sim`, `t_wall`, `agents`, and positions equal the stored position plus `antenna_offset`; (A2) the rate is the
configured one within +-30 %, `seq` rises by 1, `t_wall` does not decrease; (A3) after 1.5 s without an update the last pose keeps being sent at the same rate and `t_wall` keeps advancing; (A4) a new position is sent after the
next update. Part B runs the stack, arms agent1's PX4 and sends `takeoff`: (B1) the antenna z in `ns3/pose_applied.csv` rises, from its mean before arming to its maximum, by `MIS_TAKEOFF_ALT` (read from PX4) +-0.5 m; (B2) no gap
longer than 1.0 s between consecutive applied samples after arming. Criteria fixed before the first run on 2026-11-03.

Result (2026-11-03, single run, same hardware and versions as stage C): all checks PASS. B1: rise 2.52 m for `MIS_TAKEOFF_ALT` 2.50 m (resting antenna z 0.16 m); B2: longest gap 0.02 s over 2272 applied samples. The first full run failed
B1 because the test read `MIS_TAKEOFF_ALT` wrongly (plain `param show` prints nothing for a parameter that is not yet in use; `param show -q` prints the value); the criterion was not changed. This shows that Isaac Sim's vehicle position reaches ns-3
and follows a takeoff. It does not show that the Isaac world frame and ns-3's frame agree beyond the origin and axes assumed in `vehicle_sim.py`, and the real-time factor was not measured.

## Isaac Sim stage E: real-time factor of an Isaac Sim run

Experimental. Isaac Sim has no equivalent of Gazebo's `/stats` topic, so `tools/timehealth.py rtf_from_poses` takes the factor from the pose feed: simulated seconds (`t_sim`, Isaac's `world.current_time`) over wall seconds
(`t_wall_apply_s`, when ns-3 applied the pose) across 5 consecutive applied samples. The feed repeats the last pose while the simulator is blocked, so a stall counts as factor 0. `tools/analyze_isaac.py <log dir> <yaml>` judges a run:
the criterion is the `time.rtf_window_s` sliding mean against `time.min_rtf` over the flight window (from the first sample whose antenna z is 0.3 m above the median z of the first 10 s, to the last sample); the whole stream
(from 5 s after the first sample) is printed for information; a run with no pose samples or no flight is "run not certified". `tests/timehealth_isaac.sh [--logs DIR]` tests the function on synthetic streams (real time gives 1.0, half speed 0.5,
a 2-s stall gives 0 and fails the sliding mean, too few samples give none) and the analyzer on a real run directory. Criteria fixed before the first run on 2026-11-03; they test that a number is produced correctly, not that Isaac meets `min_rtf`.

Results (2026-11-03, single runs):
- `tests/timehealth_isaac.sh --logs <stage D run>`: all checks PASS. The measured run (one Iris, empty scene, no sensors, RTX 5060 Ti, taking off to 2.5 m): flight window (40.3 s) worst 5-s mean 0.975 (>= `min_rtf` 0.95), worst single sample 0.68; whole stream
  worst 5-s mean 0.731 (includes start-up and the wait for PX4); ns-3 lag at most 1.687 ms (limit 10 ms).
- Cross-check of the method on the Gazebo regression run (`tests/containers_leader_follower.sh`): pose-derived median 1.000, worst 5-s mean 0.997, single samples 0.80 to 1.20; Gazebo's own `/stats` factor: median 1.000, worst 5-s mean 1.000.
  The pose-derived value agrees on the sliding mean and is noisier per sample.
- The earlier stage B whole-run figures (median 0.85, min 0.55 over 10-s windows) were measured differently, over a run that included the wait for PX4; they are not comparable with the flight-window figure.
These figures are for an empty scene with no sensors; stage F adds a camera. The flight-window mean factor of the measured stage E run was 1.158 (not stated above): the figures show a lower bound being met, not real-time pacing (see the stage F finding).

## Isaac Sim stage F: camera path

Experimental. One Isaac Sim camera on the vehicle (`agents[].camera`, Isaac only), streamed over the management network to the agent container and never over the emulated radio. `tests/isaac_camera_path.sh` with
`config/experiments/isaac_camera.yaml` (camera 320 x 240 at 15 Hz configured). Criteria, fixed on 2026-11-03 before the first run (the owner accepted these thresholds): (F1) at the agent container's receiver the average
frame rate between arming and the end of the flight window is at least 10 frames/s; (F2) each frame has `width x height x 3` bytes and at least 90 % of frames are not constant (the standard deviation of a byte sample > 1.0); (F3) the ns-3 pcap
holds frames (so the check is not vacuous) and none to or from TCP port 5700 + `px4_instance`; (F4) the flight-window real-time factor, judged by `tools/analyze_isaac.py`, stays at or above `time.min_rtf` (0.95) with the camera on. Part A of the
test checks the frame protocol and receiver (`orchestrator/camera_sink.py`) without Docker. The measured frame rate and real-time factor with the camera are reported next to the stage E figure without a camera.

Results (2026-11-03, two single runs, same hardware and versions as stage C; `COSIM_ACCEPT_ISAAC_EULA=Y tests/isaac_camera_path.sh`, the second with `save_every: 100` added to the config): all checks PASS in both. Part A (protocol and receiver) passes.
- F1: 16.9 frames/s (766 frames in 45.2 s) and 15.6 frames/s (706 frames in 45.2 s) at the receiver, against 15 Hz configured in simulated time. A rate above the configured one is expected when simulated time runs faster than wall time (next point). Frames
  rendered before the receiver was connected are dropped (run 1: sequence numbers 1 to 74 missing at the start). Each frame is 230 400 bytes, so the stream is about 3.6 MB/s.
- F2: all frames 320 x 240 x 3 bytes, 100 % not constant. Saved frames (run 2, converted to PNG and looked at) show the lit scene: a red post, grey ground, horizon and sky. The offline simulator container logs MDL material compile errors for materials
  hosted on NVIDIA's asset server (observed names `Linen_Blue`, `Carpet_Gray`); the frames above show no sign of them, but material realism offline is not established.
- F3: the pcap held 9 frames and none on TCP port 5700. This is a weak check: the stream is on the management network by construction and the radio was nearly idle (one agent).
- F4 (flight-window worst 5-s mean of the real-time factor, criterion 0.95): run 1 0.986, run 2 0.952. Run 2 passed by 0.002; the run-to-run difference is larger than any difference between runs with and without the camera (stage E, no camera, one run: 0.975).
  At this resolution and rate no camera penalty is visible in single runs; none can be excluded.
- **Finding, not a criterion: Isaac Sim's simulation is not paced to wall time.** In the flight window the mean factor was 1.12 (run 1) and 1.03 (run 2); the stage E run without a camera had a mean of 1.16 and 5-s means up to 1.62. Gazebo is held at 1.000 by its own
  setting. A factor above 1 means vehicle dynamics advance faster than the wall-clock ROS 2 nodes and ns-3 expect, which the validity check (a lower bound only) does not catch. `tools/analyze_isaac.py` now prints the flight-window mean as information. Pacing the loop
  to wall time, or an upper bound on the factor, is not implemented and would need its own criterion.

## Demo with the Gazebo GUI

`demo/run_gui_demo.sh [--config YAML] [--world NAME] [--record] [--wireshark] [--no-gui] [--no-prompt]` runs the leader-follower flight with the
Gazebo GUI (default world: Baylands, whose terrain is downloaded from Gazebo Fuel on first use, about 500 MB). `demo/monitor.py <log dir>` is a live
terminal dashboard and `demo/cleanup.sh [--config YAML]` stops leftover processes and removes network plumbing after an interrupted run.
The demo flies every agent in the config. Its overview camera and the dashboard are laid out for the two-agent config: with more agents,
move the camera in the GUI; the dashboard shows only `agent1` and `agent2`.

`demo/run_container_demo.sh [--config YAML] [--no-gui] [--no-map] [--no-markers] [--no-prompt]` is the same demo on the Docker deployment ([CONTAINERS.md](CONTAINERS.md)), without sudo,
and adds a live 2D map of every agent pair's signal strength next to the Gazebo GUI. It has the same camera framing; its dashboard is the map window rather than `demo/monitor.py`.

## Reference results

Single runs on the development machine (16 cores, 31 GB), rootless (or Docker Compose where stated), Cyclone DDS, world `px4_flat`, 10 m square at 5 m altitude and
1.5 m/s. They show that the pipeline works and are not statistics.

| Run | Followers | Formation error p50 / p95 (m) | Swarm loss | Swarm latency p50 / p99 (ms) | ns-3 max lag |
|---|---|---|---|---|---|
| `five_agent_wifi.yaml` | 4 (8 to 32 m from the leader) | 1.62 to 1.65 / 1.74 to 1.79 | 0 % | 1.15 to 1.28 / 1.74 to 2.34 | 0.11 ms |
| `two_agent_wifi.yaml` | 1 (3 m) | 1.64 / 1.82 | 0 % | 1.06 / 1.44 | 0.09 ms |
| `five_agent_wifi.yaml`, Docker Compose | 4 (8 to 32 m from the leader) | 1.65 to 1.68 / 1.81 to 1.88 | 0 % | 1.37 to 1.56 / 2.10 to 2.67 | 0.57 ms |
| `two_agent_wifi.yaml`, Docker Compose | 1 (3 m) | 1.67 / 1.84 | 0 % | 1.11 / 1.57 | 0.28 ms |

The steady formation error of about 1.7 m is roughly 1.1 s of lag at 1.5 m/s; the radio is not the cause. It is consistent
with the position-loop response of PX4 combined with position-only setpoints (no velocity feed-forward); this has not been tested.
With the default receiver model all followers are inside the range of about 51 m, so latency changes little with distance. Gradual
degradation needs the `per_limited` receiver model with followers near 100 m.

Distance-sweep range (UDP loss reaching 1 % / 50 % / 99 %), from an earlier set of runs that predates the current script layout
and has not been repeated: default 51.5 m (hard step), `snr_floor` 95 m (hard step), `per_limited` 101 / 105.5 / 110.5 m.

## Known measurement caveats

- Runs are real-time. A run is only valid if the lag and real-time-factor checks pass; machines with too few cores for the number of
  vehicles will fail them.
- Do not start extra gz-transport clients (for example one `gz service` call per measurement) during a measured window; each stalls the Gazebo server and lowers the real-time factor.
- Results should be repeated and reported with variance before being cited.
