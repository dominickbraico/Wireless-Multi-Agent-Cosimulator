# Wireless Multi-Agent Cosimulator

A co-simulation framework for multi-robot systems in which robot-to-robot wireless communication is emulated by
[ns-3](https://www.nsnam.org/). Packets from unmodified ROS 2 processes cross a simulated 802.11 channel whose geometry
follows the vehicles in a physics simulator.

Each robot is a PX4 SITL vehicle with its ROS 2 nodes. It runs in its own Linux network namespace or container and
reaches other robots only through ns-3. Range limits, loss, contention and latency therefore act on the robot software
as it is.

Status: research software under active development. Interfaces and results may change.

## Contents

- [Features](#features)
- [Requirements](#requirements)
- [Quick start](#quick-start)
- [Documentation](#documentation)
- [Repository layout](#repository-layout)
- [Limitations](#limitations)

## Features

- Real-time 802.11 emulation through the ns-3 TapBridge, with positions fed from the physics simulator.
- A configurable channel and receiver model: path loss, RSSI and SNR floors, error-rate model.
- One experiment YAML as the single source of truth for a run.
- Two deployments: native network namespaces, or Docker Compose with one container per agent.
- Two flight scenarios: leader-follower formation, and mesh dropout in which links drop and recover.
- Multirotor (PX4 x500) and ground (Aion R1) vehicles in Gazebo Harmonic.
- Isolation checks: nothing flows between robots without ns-3, and only the swarm DDS domain crosses the radio.
- Run-validity checks: ns-3 real-time lag and the simulator's real-time factor.
- Analysis tools: formation error, loss, latency, a link-budget prediction and a signal-strength map of every agent
  pair.

## Requirements

| Deployment | Needs on the host |
|---|---|
| Docker | Docker with Compose |
| Native | Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, and the packages in [docs/SETUP.md](docs/SETUP.md) |

Pinned versions: ns-3 3.48 with one local patch, ROS 2 Jazzy, Gazebo Harmonic, Cyclone DDS, and the PX4 commit in
`px4/Dockerfile`.

## Quick start

Run a five-robot leader-follower flight in Docker:

```bash
docker build -t cosim-px4-runtime px4/
python3 orchestrator/cosim.py build
tests/containers_leader_follower.sh --config config/experiments/five_agent_wifi.yaml
```

Run the same flight with the Gazebo GUI and a live signal-strength map:

```bash
demo/run_container_demo.sh
```

Run the mesh-dropout scenario:

```bash
tests/containers_mesh_dropout.sh --config config/experiments/four_agent_mesh_dropout.yaml
```

Run natively, without Docker at run time (see [docs/SETUP.md](docs/SETUP.md) for the PX4 runtime):

```bash
ns3/build_ns3.sh
(cd ros2_ws && colcon build --symlink-install)
tests/run_rootless.sh tests/static_link.sh
tests/run_rootless.sh tests/leader_follower_fly.sh --config config/experiments/five_agent_wifi.yaml
```

## Documentation

| Document | Contents |
|---|---|
| [docs/SETUP.md](docs/SETUP.md) | Requirements, build, first run, troubleshooting |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Components, emulation path, DDS domains, swarm protocol, run validity |
| [docs/CONFIGURATION.md](docs/CONFIGURATION.md) | The experiment YAML: sections, agents, roles, QoS |
| [docs/CONTAINERS.md](docs/CONTAINERS.md) | Docker deployment, adding a simulator, a vehicle or a network technology, worlds |
| [docs/EXPERIMENTS.md](docs/EXPERIMENTS.md) | Tests, outputs, pass criteria, results |
| [docs/MESH_DROPOUT.md](docs/MESH_DROPOUT.md) | The mesh-dropout scenario |
| [docs/SIGNAL_MODEL.md](docs/SIGNAL_MODEL.md) | The received-power equation, its parameters and their origin |
| [docs/STYLE.md](docs/STYLE.md) | Documentation rules |
| [ns3/patches/README.md](ns3/patches/README.md) | The local ns-3 patch |

## Repository layout

| Path | Contents |
|---|---|
| `config/` | Experiment YAMLs (`experiments/`) and DDS configurations (`dds/`) |
| `ns3/` | ns-3 build script, local patch and scenarios |
| `orchestrator/` | Network wiring, process launchers, scenario drivers, Docker Compose generator (`cosim.py`) |
| `docker/` | Dockerfiles and entrypoints for the ns-3, agent and simulator containers |
| `adapters/` | Simulator adapters: Gazebo pose bridge and sweep driver, Isaac Sim scene script and pose feed |
| `ros2_ws/src/` | ROS 2 packages: `swarm_msgs`, `swarm_control`, `mesh_msgs`, `mesh_control`, `px4_offboard_control` |
| `tests/` | Acceptance and experiment scripts |
| `tools/` | Analysis, plotting, link budget, PX4 runtime setup |
| `demo/` | GUI demos and cleanup |
| `worlds/` | Gazebo worlds |
| `px4/` | Dockerfile for the PX4 runtime, and a script that fetches PX4's Gazebo models |

## Limitations

- All reported results are single runs on one machine. See [docs/EXPERIMENTS.md](docs/EXPERIMENTS.md).
- The channel model uses ns-3 default parameters and is not calibrated to hardware. See
  [docs/SIGNAL_MODEL.md](docs/SIGNAL_MODEL.md).
- Isaac Sim support is an experimental spike, not a supported simulator. See [docs/CONTAINERS.md](docs/CONTAINERS.md).
- The repository has no license file and no citation file yet.
