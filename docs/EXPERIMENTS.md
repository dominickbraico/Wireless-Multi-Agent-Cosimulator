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
| `comm_map.png` | 2D map of every agent pair's signal strength at mid-flight, dark green (strong) to red (weak) ([tools/plot_comm_map.py](../tools/plot_comm_map.py)); the values come from `ns3/pair_rssi.csv` |
| `ns3/pair_rssi.csv` | Signal strength of every ordered agent pair every `network.link_log_interval_s`: `t_s,tx,rx,distance_m,rssi_dbm,below_floor`. Evaluated by ns-3 on its channel loss chain at the current node positions, so it exists for pairs that never talk |
| `link_vs_distance.png` | Swarm latency and loss against mean distance to the leader (more than one follower) |
| `swarm_follower_<agent>.csv` | Per-message sequence number, latency and gap for that follower |
| `<agent>_flight.log`, `swarm_<agent>.log` | Node logs |
| `ns3/` | ns-3 summary, applied poses, RSSI, lag and pcap files |
| `firewall_counters.txt` | What the radio firewall blocked, per agent |

Pass criteria: every vehicle reaches altitude; the leader lands; each follower holds its formation (horizontal error p95 at most
5 m while the leader moves, a threshold fixed before the first run), receives the stream with less than 5 % sequence loss and
lands when the link goes away; no domain-0, XRCE or PX4-topic traffic and only domain-42 traffic on the channel; ns-3 lag within
`max_lag_ms`; Gazebo real-time factor within `min_rtf`. Formation error is measured against Gazebo ground truth.

## Demo with the Gazebo GUI

`demo/run_gui_demo.sh [--config YAML] [--world NAME] [--record] [--wireshark] [--no-gui] [--no-prompt]` runs the leader-follower flight with the
Gazebo GUI (default world: Baylands, whose terrain is downloaded from Gazebo Fuel on first use, about 500 MB). `demo/monitor.py <log dir>` is a live
terminal dashboard and `demo/cleanup.sh [--config YAML]` stops leftover processes and removes network plumbing after an interrupted run.
The demo flies every agent in the config. Its overview camera and the dashboard are laid out for the two-agent config: with more agents,
move the camera in the GUI; the dashboard shows only `agent1` and `agent2`.

`demo/run_container_demo.sh [--config YAML] [--no-gui] [--no-map] [--no-prompt]` is the same demo on the Docker deployment ([CONTAINERS.md](CONTAINERS.md)), without sudo,
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
