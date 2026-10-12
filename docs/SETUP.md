# Setup

Developed and tested on Ubuntu 24.04 (Linux 6.8).

## Requirements

- ROS 2 Jazzy and `ros-jazzy-rmw-cyclonedds-cpp`
- Gazebo Harmonic
- Docker (only to build the PX4 image from which the PX4 runtime is extracted)
- `iproute2`, `nftables` (`nft`), `tcpdump`, `libyaml-cpp-dev`, a C++ toolchain and CMake (for ns-3)
- Python 3 with `matplotlib` and `PyYAML`
- Root, or the rootless wrapper `tests/run_rootless.sh` (needs unprivileged user namespaces)

## Build

1. **ns-3 3.48 and the scenarios** (downloads ns-3, applies the local patch, builds into `~/dev/ns-3-build`):
   ```
   ns3/build_ns3.sh
   ```
   Re-run it only when the C++ under `ns3/scenarios/` or `ns3/patches/` changes. Tests find the binary under `~/dev/ns-3-build`; override with `NS3=<path>`.
2. **ROS 2 workspace** (`swarm_msgs`, `swarm_control`, `px4_offboard_control`):
   ```
   (cd ros2_ws && colcon build --symlink-install)
   ```
3. **Network-only tests** (`static_link`, `ros2_isolation`, `swarm_leader_follower`, `pose_feed`, `distance_sweep`) need nothing more.
4. **Flight tests** also need the PX4 runtime:
   - Build the PX4 image: `docker build -t cosim-px4-runtime px4/` (a long build; it compiles PX4).
   - Run `tools/setup_px4_runtime.sh`. It extracts PX4 SITL, the Micro XRCE-DDS Agent and `px4_msgs` into `~/dev/px4-runtime` (override with `PX4_RUNTIME`).
   - Run `px4/fetch_gazebo_models.sh` to download PX4's Gazebo models and server configuration into `~/.simulation-gazebo`.
   - The PX4 version is pinned in `px4/Dockerfile`. See `px4/README.md`.
   - `tools/setup_px4_runtime.sh` has not yet been validated on a clean machine.

## Run

```
tests/run_rootless.sh tests/static_link.sh
tests/run_rootless.sh tests/leader_follower_fly.sh --config config/experiments/five_agent_wifi.yaml
sudo tests/leader_follower_fly.sh
```

See [EXPERIMENTS.md](EXPERIMENTS.md) for what each test checks and [CONFIGURATION.md](CONFIGURATION.md) for the experiment file.

## Troubleshooting

- **PX4 starts but Gazebo is not found.** ROS 2 ships a stub `gz` that shadows the real binary and PX4's start-up script calls `gz --versions`.
  Every process that starts PX4 needs `/usr/bin` first on `PATH` (the scripts do this).
- **PX4 fails with "px4_daemon: error connecting to socket".** Stale `/tmp/px4-sock-*` and `/tmp/px4_lock-*` files from an interrupted run. `demo/cleanup.sh` removes them;
  each PX4 started by `run_stack.py` uses a private `/tmp`.
- **Leftover namespaces, bridges or processes after an interrupt.** Run `demo/cleanup.sh`.
- **Run reported invalid.** The ns-3 lag or Gazebo real-time-factor check failed; reduce the number of vehicles or free CPU.
