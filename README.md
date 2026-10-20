# Wireless Multi-Agent Cosimulator

A co-simulation framework for multi-robot systems in which robot-to-robot wireless communication is emulated by
[ns-3](https://www.nsnam.org/). Real packets from real ROS 2 processes cross a simulated 802.11 channel, independent of the
physics simulator (Gazebo Harmonic; other simulators can be added through the pose-feed interface).

Each robot (a PX4 SITL vehicle and ROS 2 nodes) runs in its own Linux network namespace and can reach other robots only through ns-3,
so channel effects such as range limits, loss, contention and latency act on the unmodified robot software.

**Status:** early research software under active development. Interfaces and results may change.

## Features

- Real-time 802.11a emulation through ns-3 TapBridge, with positions fed live from the physics simulator
- Configurable receiver and channel model (path loss, RSSI/SNR floors, error-rate model); range predicted by `tools/link_budget.py`
- N-robot leader-follower flight with PX4 SITL and ROS 2, all inter-robot traffic through the emulated channel
- Isolation checks: nothing flows between robots without ns-3; only the swarm DDS domain crosses the radio
- Run-validity checks (ns-3 real-time lag, Gazebo real-time factor) and per-follower analysis of formation error, loss and latency
- One experiment YAML as the single source of truth for a run
- Runs natively (network namespaces) or fully in Docker Compose: one container per agent, one for ns-3, one for the physics simulator ([docs/CONTAINERS.md](docs/CONTAINERS.md))
- Selectable radio technology (802.11a/g/n/ac/ax today; new technologies plug in as ns-3 scenarios) and a 2D map of the signal strength of every agent pair (`tools/plot_comm_map.py`)

## Quick start

Requirements are listed in [docs/SETUP.md](docs/SETUP.md) (Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic).

```
ns3/build_ns3.sh                                  # fetch, patch and build ns-3 3.48 and the scenarios
(cd ros2_ws && colcon build --symlink-install)    # ROS 2 packages
tests/run_rootless.sh tests/static_link.sh        # network-only check, no sudo needed
```

Leader-follower flight with five robots (needs the PX4 runtime, see [docs/SETUP.md](docs/SETUP.md)):

```
tests/run_rootless.sh tests/leader_follower_fly.sh --config config/experiments/five_agent_wifi.yaml
```

The same flight in Docker Compose, with nothing but Docker on the host ([docs/CONTAINERS.md](docs/CONTAINERS.md)):

```
python3 orchestrator/cosim.py build
tests/containers_leader_follower.sh --config config/experiments/five_agent_wifi.yaml
```

## Documentation

| Document | Contents |
|---|---|
| [docs/SETUP.md](docs/SETUP.md) | Requirements, build, running, troubleshooting |
| [docs/CONTAINERS.md](docs/CONTAINERS.md) | Docker deployment, container networking, the physics-simulator container, adding Isaac Sim, changing the network technology |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Components, emulation path, DDS domains, swarm protocol, limitations |
| [docs/CONFIGURATION.md](docs/CONFIGURATION.md) | The experiment YAML, agents, roles, positions |
| [docs/EXPERIMENTS.md](docs/EXPERIMENTS.md) | Tests, outputs, pass criteria, reference results |
| [ns3/patches/README.md](ns3/patches/README.md) | The local ns-3 patch |

## Repository layout

| Path | Contents |
|---|---|
| `config/` | Experiment YAMLs (`experiments/`) and DDS configurations (`dds/`) |
| `ns3/` | ns-3 build script, local patch, scenarios (`wifi_adhoc_tap`, `per_table`) |
| `orchestrator/` | Network wiring, stack launcher, fleet roles, Docker Compose generator and driver (`cosim.py`) |
| `docker/` | Dockerfiles and entrypoints for the ns-3, agent and simulator containers |
| `adapters/gazebo/` | Pose bridge and distance-sweep driver |
| `ros2_ws/src/` | `swarm_msgs`, `swarm_control`, `px4_offboard_control` |
| `tests/` | Acceptance and experiment scripts |
| `tools/` | Analysis, plotting, link-budget and PX4 runtime setup tools |
| `demo/` | Gazebo GUI demo and cleanup |
| `worlds/` | Gazebo worlds |
| `px4/` | Dockerfile for the PX4 runtime and a script that fetches PX4's Gazebo models |

