# Configuration

An experiment is one YAML file in `config/experiments/`. Every component (wiring, ns-3, pose bridge, PX4 launcher, analysis)
reads the same file, so it is the single source of truth for a run. Copy one, edit it, and pass it with `--config`:

```
tests/run_rootless.sh tests/leader_follower_fly.sh --config config/experiments/my_run.yaml
```

Editing a YAML never requires rebuilding ns-3: the file is read each time ns-3 starts.

## Provided experiments

| File | Purpose |
|---|---|
| `two_agent_wifi.yaml` | Two agents; default receiver model (the range is set by an RSSI floor, a hard step at about 51 m) |
| `two_agent_wifi_snr_floor.yaml` | Same, with the RSSI floor lowered so an SNR floor sets the range (about 95 m) |
| `two_agent_wifi_per_limited.yaml` | Both floors lowered so the packet error rate sets the range (gradual, about 100 to 110 m) |
| `five_agent_wifi.yaml` | Five agents, one leader and four followers spaced 8 m apart |
| `two_agent_wifi_80211g.yaml`, `two_agent_wifi_80211n.yaml` | `two_agent_wifi.yaml` with the radio technology changed to 802.11g (2.4 GHz) and 802.11n (5 GHz) |

## Sections

| Section | Contents |
|---|---|
| `experiment` | `name`, `duration_s`, `log_dir` |
| `time` | `ns3_sync_mode`, `ns3_hard_limit_s`, `max_lag_ms` (run flagged invalid above this), `min_rtf`, `rtf_window_s` |
| `network.technology` | Radio technology = which ns-3 scenario runs (default and only built-in: `wifi`). See [CONTAINERS.md](CONTAINERS.md), "Changing the networking technology" |
| `network.wifi` | `standard` (`80211a`, `80211g`, `80211n`, `80211ac`, `80211ax`), `band`, `mac`, `rate_manager`, `data_mode`, `control_mode`, `tx_power_dbm`, `error_rate_model` (`TableBased` or `Nist`), `retry_limit`, `phy.*` receiver parameters |
| `network.channel` | `loss` (propagation loss models), `delay` |
| `network.link_log_interval_s` | Optional (default 0.5, 0 = off): how often ns-3 logs the signal strength of every ordered agent pair to `pair_rssi.csv` (simulated seconds) |
| `network.pose` | Pose feed: `transport`, `listen_addr`, `port`, `rate_hz`. Omit the section to run with static positions |
| `network.radio_subnet` | Subnet of the emulated radio, e.g. `10.10.0.0/24` |
| `network.radio_firewall` | `enabled`, `allow_udp_ports` (the RTPS ports of the swarm DDS domain) |
| `network.mgmt` | Unemulated management network between the simulator and the robots |
| `agents` | One entry per robot (below) |
| `containers` | Optional, Docker deployment only: `ns3_mgmt_ip`, `link_subnet_base` (below) |
| `swarm.qos.leader_state`, `swarm.qos.peer_state` | Optional: DDS QoS of the radio-side topics `/swarm/leader_state` (leader-follower) and `/mesh/peer_state` (mesh dropout) (below). Default `best_effort`, `volatile`, depth 1 (10 for `peer_state`) |
| `scenario` | Optional: `type` (`leader_follower`, the default, or `mesh_dropout`) and, for the latter, the `mesh_dropout` keys: see [MESH_DROPOUT.md](MESH_DROPOUT.md) |
| `simulator` | `type`, `world`, `world_name`, `pose_topic`, `gz_partition`, `px4_world_name` |

The receiver parameters in `network.wifi.phy` (noise figure, preamble-detection minimum RSSI and SNR, CCA energy-detect
threshold, sensitivity) determine which mechanism limits the range; each corresponds to a measurable hardware quantity.

## Agents

```yaml
- {name: agent2, radio_ip: 10.10.0.2, mgmt_ip: 10.20.0.12, px4_instance: 1, px4_gz_model: x500_1,
   spawn_pose: [0, 8, 0.3, 0], tap: tap-a2, bridge: br-a2, antenna_offset: [0, 0, 0.1]}
```

| Field | Meaning |
|---|---|
| `name` | Agent name; the namespace is `ns-<name>` |
| `radio_ip`, `mgmt_ip` | Addresses on the emulated radio and on the management network |
| `px4_instance` | PX4 instance number (`-i N`); also selects the `/px4_N` topic prefix and MAVLink system id N+1 |
| `px4_gz_model` | Gazebo model name the pose bridge tracks (`x500_<instance>` for PX4 vehicles) |
| `spawn_pose` | `[x, y, z, yaw]`: where PX4 spawns the vehicle (world frame, metres) |
| `antenna_offset` | Antenna position relative to the model origin (default `[0, 0, 0]`) |
| `initial_position` | Optional. Start position of the agent's ns-3 node (see below) |
| `tap`, `bridge` | Names of the TAP device and Linux bridge (interface names are limited to 15 characters) |
| `role` | Optional: `leader` or `follower` |
| `follower_offset` | Optional: `[x, y, z]` formation offset from the leader (world frame) |

### Docker deployment (`containers`)

The container deployment ([CONTAINERS.md](CONTAINERS.md)) derives everything from the sections above. Two optional keys exist, both for avoiding address clashes:

```yaml
containers:
  ns3_mgmt_ip: 10.20.0.2           # the ns3 container on the management network (default: first address not used by the simulator or an agent)
  link_subnet_base: 10.30.0.0/16   # agent i gets the i-th /24 of this block as its private link network to ns-3
```

In a container run, `network.mgmt.bridge_ip` is the simulator container's address, `network.pose.listen_addr` is replaced by the ns3
container's address in the copy of the YAML the containers read (`<log dir>/compose/experiment_run.yaml`), and `bridge` and `tap` still name the bridge and TAP inside the ns3 container.

### ns-3 start position

ns-3 needs a position for every node at start-up. It is replaced by the first Gazebo pose when the pose feed runs, and it is
the position for the whole run when it does not (network-only tests).

- If `initial_position` is omitted, ns-3 uses `spawn_pose` x, y, z plus `antenna_offset`. Flight configs should omit it.
- If given, it takes precedence. `two_agent_wifi.yaml` uses it to place agent 2 at a fixed 10 m for static link tests.
- If neither is present, ns-3 aborts at start-up naming the agent.

### Roles and formation

Leader-follower runs use `orchestrator/fleet.py` (`orchestrator/fleet.py <yaml>` prints the resolution):

- The leader is the agent with `role: leader`; without any role, the first agent. Exactly one leader is supported.
- All other agents are followers.
- A follower's offset (follower minus leader, world frame) is its `follower_offset` if given, otherwise the difference of the
  two `spawn_pose` values, so each follower holds the formation it was spawned in. `OFFSET=x,y,z` forces one offset on every follower.

### Swarm message QoS (`swarm.qos`)

```yaml
swarm:
  qos:
    leader_state:                 # /swarm/leader_state, the only topic that crosses the emulated channel
      reliability: best_effort    # best_effort | reliable
      durability: volatile        # volatile | transient_local
      depth: 1                    # keep-last depth, integer >= 1
```

All keys are optional; the values shown are the defaults. The leader and every follower get the same profile (a best-effort
publisher never matches a reliable subscriber, so differing profiles deliver nothing). Invalid values are rejected before the
run starts. The profile in effect is printed in each `swarm_*.log` (`leader_state QoS: ...`).

Choose by what the message means, not by default. `best_effort` suits periodic state where only the newest sample matters:
loss stays visible (sequence gaps) and no retransmission load is added to the shared channel. `reliable` suits a message other
agents plan around, because a lost one is otherwise invisible; on 802.11 it converts loss into latency, so check `latency_ms`
in `swarm_follower_*.csv`. `transient_local` hands a late-joining agent the last sample. Samples larger than one 1500-byte frame
are fragmented, and under `best_effort` losing one fragment loses the sample. Further topics (for example a committed
trajectory) are added as further keys under `swarm.qos`; `leader_state` and `peer_state` exist today. `peer_state` (the mesh-dropout heartbeat) defaults to depth 10, not 1: the topic has no key and
several publishers, so a subscriber's keep-last history is shared by all senders and depth 1 lets simultaneous heartbeats overwrite each other ([MESH_DROPOUT.md](MESH_DROPOUT.md)).

Applies to `tests/leader_follower_fly.sh`, `tests/containers_mesh_dropout.sh` (`peer_state`) and the container deployment (the demos use those). `tests/swarm_leader_follower.sh`
and the sweep scripts start the node by hand and use its defaults.

## Adding robots

Append entries to `agents` with unique names, addresses, `px4_instance`, `tap` and `bridge`, and spawn poses that do not overlap.
Nothing else needs editing. The radio is a shared medium, so contention emerges from the ns-3 model. Each additional PX4 instance adds
real-time load; see "Run validity" in [ARCHITECTURE.md](ARCHITECTURE.md).

## DDS configuration

`config/dds/cyclonedds_agent.xml` binds the swarm domain (42) to `radio0`; `cyclonedds_radio.xml` does the same for all domains
and is used by the network-only tests; `fastdds_loopback.xml` restricts the XRCE agent to the loopback interface. Cyclone DDS
(`rmw_cyclonedds_cpp`) is the default RMW of the test scripts.
