# Setup

How to build and run the native deployment. For the Docker deployment, which needs only Docker on the host, see
[CONTAINERS.md](CONTAINERS.md).

## Requirements

Developed and tested on Ubuntu 24.04 (Linux 6.8).

| Requirement | Used for |
|---|---|
| ROS 2 Jazzy and `ros-jazzy-rmw-cyclonedds-cpp` | Robot software and the default DDS implementation |
| Gazebo Harmonic | Physics simulation |
| Docker | Building the PX4 image from which the PX4 runtime is extracted |
| `iproute2`, `nftables`, `tcpdump` | Network wiring, firewall, packet capture |
| `libyaml-cpp-dev`, a C++ toolchain, CMake | Building ns-3 and the scenarios |
| Python 3 with `matplotlib` and `PyYAML` | Orchestration and analysis |
| Root, or unprivileged user namespaces | Creating network namespaces (`tests/run_rootless.sh` uses the latter) |

## Build

1. Build ns-3 3.48 and the scenarios. The script downloads ns-3, applies the local patch and builds into
   `~/dev/ns-3-build`:

   ```bash
   ns3/build_ns3.sh
   ```

   Re-run it only when the C++ under `ns3/scenarios/` or the patch changes. Tests find the build under
   `~/dev/ns-3-build`; override with `NS3=<path>`.

2. Build the ROS 2 workspace:

   ```bash
   (cd ros2_ws && colcon build --symlink-install)
   ```

   The network-only tests (`static_link`, `ros2_isolation`, `swarm_leader_follower`, `pose_feed`, `distance_sweep`)
   need nothing more.

3. For flight tests, build the PX4 image, extract the runtime and fetch PX4's Gazebo models:

   ```bash
   docker build -t cosim-px4-runtime px4/
   tools/setup_px4_runtime.sh
   px4/fetch_gazebo_models.sh
   ```

   The first command compiles PX4 and takes a long time. The second extracts PX4 SITL, the Micro XRCE-DDS Agent and
   `px4_msgs` into `~/dev/px4-runtime`; override with `PX4_RUNTIME=<path>`. The third downloads into
   `~/.simulation-gazebo`. The PX4 version is pinned in `px4/Dockerfile`.

## Quick start

Run a network-only check, then a five-robot flight, without sudo:

```bash
tests/run_rootless.sh tests/static_link.sh
tests/run_rootless.sh tests/leader_follower_fly.sh --config config/experiments/five_agent_wifi.yaml
```

Run a flight as root instead:

```bash
sudo tests/leader_follower_fly.sh
```

[EXPERIMENTS.md](EXPERIMENTS.md) describes what each test checks. [CONFIGURATION.md](CONFIGURATION.md) describes the
experiment file.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| PX4 starts but does not find Gazebo | ROS 2 ships a stub `gz` that shadows the real binary | Put `/usr/bin` first on `PATH` for every process that starts PX4. The scripts do this. |
| `px4_daemon: error connecting to socket` | Stale `/tmp/px4-sock-*` and `/tmp/px4_lock-*` files from an interrupted run | Run `demo/cleanup.sh`. |
| Leftover namespaces, bridges or processes | An interrupted run | Run `demo/cleanup.sh`. |
| Run reported invalid | The ns-3 lag or real-time-factor check failed | Reduce the number of vehicles or free CPU. |

## Limitations

- `tools/setup_px4_runtime.sh` is not tested on a clean machine.
