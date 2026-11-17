# Configuration

Reference for the experiment YAML. One file in `config/experiments/` describes a run, and every component reads the
same file: wiring, ns-3, pose bridge, PX4 launcher and analysis.

## Contents

- [Usage](#usage)
- [Provided experiments](#provided-experiments)
- [Sections](#sections)
- [Agents](#agents)
- [Start position in ns-3](#start-position-in-ns-3)
- [Roles and formation](#roles-and-formation)
- [Message QoS](#message-qos)
- [Docker deployment keys](#docker-deployment-keys)
- [DDS configuration files](#dds-configuration-files)
- [Adding robots](#adding-robots)

## Usage

Copy a provided file, edit it and pass it with `--config`:

```bash
tests/run_rootless.sh tests/leader_follower_fly.sh --config config/experiments/my_run.yaml
```

Editing a YAML never requires rebuilding ns-3. ns-3 reads the file each time it starts.

## Provided experiments

| File | Agents | Purpose |
|---|---|---|
| `two_agent_wifi.yaml` | 2 x500 | Default receiver model: an RSSI floor sets the range, a hard step at about 51 m |
| `two_agent_wifi_snr_floor.yaml` | 2 x500 | RSSI floor lowered, so an SNR floor sets the range (about 95 m) |
| `two_agent_wifi_per_limited.yaml` | 2 x500 | Both floors lowered, so the packet error rate sets the range (about 100 m to 110 m) |
| `two_agent_wifi_reliable.yaml` | 2 x500 | `two_agent_wifi.yaml` with reliable QoS on `leader_state` |
| `two_agent_wifi_80211g.yaml` | 2 x500 | `two_agent_wifi.yaml` on 802.11g (2.4 GHz) |
| `two_agent_wifi_80211n.yaml` | 2 x500 | `two_agent_wifi.yaml` on 802.11n (5 GHz) |
| `five_agent_wifi.yaml` | 5 x500 | One leader and four followers spaced 8 m apart |
| `five_agent_wifi_baylands.yaml` | 5 x500 | The same formation with spawn heights that match the Baylands terrain |
| `four_agent_mesh_dropout.yaml` | 4 x500 | Mesh-dropout scenario, flat world, transmit power 4 dBm |
| `four_agent_mesh_dropout_baylands.yaml` | 4 x500 | Mesh-dropout scenario in Baylands |
| `drone_rover_mesh_baylands.yaml` | 1 x500, 1 R1 rover | Mesh-dropout scenario with a mixed fleet |
| `two_drone_two_rover_mesh_baylands.yaml` | 2 x500, 2 R1 rovers | Mesh-dropout scenario with a mixed fleet of four |
| `plane_drone_rover_mesh_baylands.yaml` | 1 RC Cessna, 2 x500, 2 R1 rovers | Fixed-wing in the mesh-dropout scenario (work in progress, no passing run) |
| `rover_two_agent.yaml` | 2 R1 rovers | Rover drive test |
| `isaac_one_agent.yaml` | 1 Iris | Isaac Sim spike (experimental) |
| `isaac_camera.yaml` | 1 Iris | Isaac Sim spike with a camera (experimental) |
| `isaac_four_agent_mesh_dropout.yaml` | 4 Iris | Mesh-dropout scenario in Isaac Sim (experimental; does not hold real time) |

The range figures come from the distance sweep reported in [EXPERIMENTS.md](EXPERIMENTS.md).

## Sections

| Section | Keys |
|---|---|
| `experiment` | `name`, `duration_s`, `log_dir` |
| `time` | `ns3_sync_mode`, `ns3_hard_limit_s`, `max_lag_ms`, `min_rtf`, `rtf_window_s` |
| `network.technology` | Radio technology, which selects the ns-3 scenario. Default and only built-in value: `wifi` |
| `network.wifi` | `standard`, `band`, `mac`, `rate_manager`, `data_mode`, `control_mode`, `tx_power_dbm`, `error_rate_model`, `retry_limit`, `phy.*` |
| `network.channel` | `loss` (list of propagation loss models), `delay` |
| `network.link_log_interval_s` | Interval of the pair signal log `pair_rssi.csv` in simulated seconds. Default 0.5; 0 disables it |
| `network.pose` | Pose feed: `transport`, `listen_addr`, `port`, `rate_hz`. Omit the section to run with static positions |
| `network.radio_subnet` | Subnet of the emulated radio, for example `10.10.0.0/24` |
| `network.radio_firewall` | `enabled`, `allow_udp_ports` (the RTPS ports of the swarm DDS domain) |
| `network.mgmt` | Management network between the simulator and the robots: `bridge`, `subnet`, `bridge_ip` |
| `agents` | One entry per robot. See [Agents](#agents) |
| `swarm.qos` | QoS of the radio topics. See [Message QoS](#message-qos) |
| `scenario` | `type` and the keys of that scenario. See the table below |
| `simulator` | `type` and the keys of that simulator. See the table below |
| `containers` | Docker deployment only. See [Docker deployment keys](#docker-deployment-keys) |

Values of `network.wifi`:

| Key | Values |
|---|---|
| `standard` | `80211a` (default), `80211g`, `80211n`, `80211ac`, `80211ax` |
| `band` | `5GHZ` or `2_4GHZ`. Default `5GHZ`, or `2_4GHZ` for `80211g` |
| `error_rate_model` | `TableBased` or `Nist` |
| `retry_limit` | Number of transmission attempts; `null` keeps the ns-3 default |
| `phy.*` | `rx_noise_figure_db`, `preamble_detection_min_rssi_dbm`, `preamble_detection_snr_db`, `cca_ed_threshold_dbm`, `rx_sensitivity_dbm` |

The receiver parameters in `network.wifi.phy` decide which mechanism limits the range. Each corresponds to a measurable
hardware quantity. [SIGNAL_MODEL.md](SIGNAL_MODEL.md) lists the defaults and their origin.

Values of `time`:

| Key | Meaning |
|---|---|
| `ns3_sync_mode` | `BestEffort` or `HardLimit`: how ns-3's real-time simulator reacts to falling behind |
| `ns3_hard_limit_s` | Tolerated lag in `HardLimit` mode |
| `max_lag_ms` | A run is invalid when the ns-3 lag exceeds this |
| `min_rtf` | A run is invalid when the real-time factor falls below this |
| `rtf_window_s` | Length of the sliding mean used for `min_rtf` |

Values of `scenario`:

| `type` | Behaviour | Keys |
|---|---|---|
| `leader_follower` (default) | One leader flies a square; followers hold a formation | None. See [Roles and formation](#roles-and-formation) |
| `mesh_dropout` | Agents fly seeded paths so that links drop and recover | `scenario.mesh_dropout.*`, listed in [MESH_DROPOUT.md](MESH_DROPOUT.md) |
| `rover_drive` | Every agent is a rover and drives a square | `scenario.rover_drive.side_m`, `speed_mps`, `acceptance_radius_m` |

Values of `simulator`:

| `type` | Keys |
|---|---|
| `gazebo` | `world`, `world_name`, `pose_topic`, `gz_partition`, `px4_world_name` |
| `isaac` (experimental) | `world_name`, a label only: the scene is a fixed ground box. See [CONTAINERS.md](CONTAINERS.md) |

## Agents

An agent entry looks like this:

```yaml
- {name: agent2, radio_ip: 10.10.0.2, mgmt_ip: 10.20.0.12, px4_instance: 1, px4_gz_model: x500_1,
   spawn_pose: [0, 8, 0.3, 0], tap: tap-a2, bridge: br-a2, antenna_offset: [0, 0, 0.1]}
```

| Field | Required | Meaning |
|---|---|---|
| `name` | Yes | Agent name. The native namespace is `ns-<name>` |
| `radio_ip` | Yes | Address on the emulated radio |
| `mgmt_ip` | Yes | Address on the management network |
| `px4_instance` | Yes | PX4 instance number *N*. Selects the `/px4_N` topic prefix and MAVLink system id *N* + 1 |
| `px4_gz_model` | Yes | Gazebo model the pose bridge tracks: `<model>_<px4_instance>`, for example `x500_1` |
| `spawn_pose` | Yes | `[x, y, z, yaw]`: where the vehicle spawns, world frame, metres |
| `tap`, `bridge` | Yes | Names of the TAP device and Linux bridge, at most 15 characters each |
| `antenna_offset` | No | Antenna position relative to the model origin. Default `[0, 0, 0]` |
| `initial_position` | No | Start position of the agent's ns-3 node. See [Start position in ns-3](#start-position-in-ns-3) |
| `vehicle` | No | Vehicle type: `x500` (default), `r1_rover`, `iris` or `rc_cessna`. Docker deployment only |
| `role` | No | `leader` or `follower` |
| `follower_offset` | No | `[x, y, z]` formation offset from the leader, world frame |
| `camera` | No | `{width, height, rate_hz, save_every}`. `simulator.type: isaac` only |

Notes on `vehicle`:

- `x500` is a multirotor in Gazebo, `r1_rover` is the Aion R1 ground vehicle in Gazebo, and `iris` is the Pegasus
  multirotor in Isaac Sim. `orchestrator/vehicles.py` holds the registry.
- A vehicle with no definition for the chosen `simulator.type` is refused by name before anything starts.
- `vehicle` selects the PX4 airframe and the simulator model, and must agree with `px4_gz_model`.

Notes on `camera`:

- `width`, `height` and `rate_hz` are required and positive. `save_every: N` also saves every Nth frame as a PPM file.
- The frames are raw 8-bit RGB sent over the management network to the agent container, on TCP port 5700 +
  `px4_instance`. They are not a ROS 2 topic.
- Outputs: `camera_<agent>.csv` and, with `save_every`, `camera_<agent>_frames/`.

## Start position in ns-3

ns-3 needs a position for every node at start-up. The first pose from the simulator replaces it. In network-only tests,
which have no pose feed, it is the position for the whole run.

| Case | Position used |
|---|---|
| `initial_position` is given | `initial_position` |
| `initial_position` is omitted | `spawn_pose` x, y, z plus `antenna_offset` |
| Neither is present | ns-3 aborts at start-up and names the agent |

Flight configurations omit `initial_position`. `two_agent_wifi.yaml` uses it to place agent 2 at a fixed 10 m for the
static link tests.

## Roles and formation

`orchestrator/fleet.py` resolves the roles of a leader-follower run. Print the resolution:

```bash
python3 orchestrator/fleet.py config/experiments/five_agent_wifi.yaml
```

- The leader is the agent with `role: leader`. Without any role, it is the first agent. Exactly one leader is supported.
- All other agents are followers.
- A follower's offset (follower minus leader, world frame) is its `follower_offset`. Without one, it is the difference
  of the two `spawn_pose` values, so each follower holds the formation it spawns in.
- `OFFSET=x,y,z` in the environment forces one offset on every follower.

## Message QoS

`swarm.qos.<topic>` sets the DDS QoS of a radio topic. All keys are optional:

```yaml
swarm:
  qos:
    leader_state:
      reliability: best_effort    # best_effort | reliable
      durability: volatile        # volatile | transient_local
      depth: 1                    # keep-last depth, integer >= 1
```

| Topic key | ROS topic | Scenario | Default |
|---|---|---|---|
| `leader_state` | `/swarm/leader_state` | Leader-follower | `best_effort`, `volatile`, depth 1 |
| `peer_state` | `/mesh/peer_state` | Mesh dropout | `best_effort`, `volatile`, depth 10 |

Every agent gets the same profile, because a best-effort publisher never matches a reliable subscriber. Invalid values
are rejected before the run starts. Each radio node prints the profile in effect in its log.

Choose the profile by what the message means:

| Setting | Suits | Effect on the channel |
|---|---|---|
| `best_effort` | Periodic state where only the newest sample matters | Loss stays visible as sequence gaps; no retransmission load |
| `reliable` | A message other agents plan around | Loss becomes latency; check `latency_ms` in the follower CSV |
| `transient_local` | State that a late-joining agent needs | The last sample is resent to new subscribers |

A sample larger than one 1500-byte frame is fragmented. Under `best_effort`, losing one fragment loses the sample.

`peer_state` defaults to depth 10 because the topic has no key and several publishers. A subscriber's keep-last history
is then shared by all senders, and depth 1 lets simultaneous heartbeats overwrite each other. See
[MESH_DROPOUT.md](MESH_DROPOUT.md).

The profile applies to `tests/leader_follower_fly.sh`, `tests/containers_mesh_dropout.sh` and the Docker deployment.
`tests/swarm_leader_follower.sh` and the sweep scripts start the node directly and use its defaults.

## Docker deployment keys

The Docker deployment derives everything from the sections above. Two optional keys avoid address clashes:

```yaml
containers:
  ns3_mgmt_ip: 10.20.0.2           # address of the ns3 container on the management network
  link_subnet_base: 10.30.0.0/16   # agent i gets the i-th /24 of this block as its private link to ns-3
```

In a container run:

- `network.mgmt.bridge_ip` is the simulator container's address.
- `network.pose.listen_addr` is replaced by the ns3 container's address in the copy of the YAML that the containers
  read, `<log dir>/compose/experiment_run.yaml`.
- `bridge` and `tap` name the bridge and TAP inside the ns3 container.

## DDS configuration files

| File in `config/dds/` | Purpose |
|---|---|
| `cyclonedds_agent.xml` | Binds the swarm domain (42) to `radio0` |
| `cyclonedds_mesh.xml` | The same for the mesh-dropout scenario, with a 1 s participant announcement interval |
| `cyclonedds_radio.xml` | Binds all domains to `radio0`; used by the network-only tests |
| `fastdds_loopback.xml` | Restricts the XRCE agent to the loopback interface |

Cyclone DDS (`rmw_cyclonedds_cpp`) is the default RMW of the test scripts.

## Adding robots

Append entries to `agents` with unique names, addresses, `px4_instance`, `tap` and `bridge`, and spawn poses that do not
overlap. Nothing else needs editing.

The radio is a shared medium, so contention emerges from the ns-3 model. Each additional PX4 instance adds real-time
load. See "Run validity" in [ARCHITECTURE.md](ARCHITECTURE.md).
