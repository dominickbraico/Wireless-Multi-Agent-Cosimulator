#!/usr/bin/env bash
# Extract the PX4 SITL runtime from the PX4 Docker image (px4/Dockerfile) and build px4_msgs, for running PX4 natively
# inside the per-agent network namespaces. Output: ~/dev/px4-runtime (outside the repo; override with PX4_RUNTIME).
#   tools/setup_px4_runtime.sh [image]      default image: cosim-px4-runtime:latest (docker build -t cosim-px4-runtime px4/)
# NOTE: not yet validated on a clean machine.
set -eu
# under plain sudo HOME may be /root; use the invoking user's home so tool paths resolve
[ -n "${SUDO_USER:-}" ] && HOME="$(getent passwd "$SUDO_USER" | cut -d: -f6)"
IMAGE="${1:-cosim-px4-runtime:latest}"
RT="${PX4_RUNTIME:-$HOME/dev/px4-runtime}"
mkdir -p "$RT/ws/src"; cd "$RT"
cid=$(docker create "$IMAGE")
trap 'docker rm "$cid" >/dev/null' EXIT
docker cp "$cid:/opt/PX4-Autopilot/build/px4_sitl_default" ./px4_sitl_default     # PX4 SITL binary + etc/ (565 MB)
docker cp "$cid:/usr/local/bin/MicroXRCEAgent" ./
mkdir -p lib && docker cp "$cid:/usr/local/lib/." ./lib/                          # agent + Fast DDS libs
docker cp "$cid:/opt/ros2_ws/src/px4_msgs" ws/src/px4_msgs                        # pinned by the image (72fcfaa...)
rm -rf ws/src/px4_msgs/.git
set +u; source /opt/ros/jazzy/setup.bash; set -u
(cd ws && colcon build --packages-select px4_msgs)                                # ~2.5 min; the extracted install is NOT relocatable
echo "PX4 runtime ready in $RT. Gazebo models/server config are expected in ~/.simulation-gazebo (px4/fetch_gazebo_models.sh creates them)."
