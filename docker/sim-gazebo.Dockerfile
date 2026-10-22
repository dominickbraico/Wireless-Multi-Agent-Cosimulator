# syntax=docker/dockerfile:1
#
# The physics-simulator container for Gazebo Harmonic: headless gz server, PX4's Gazebo models and the pose adapter
# that feeds ground-truth positions to ns-3. Build from the repository root:
#   docker build -f docker/sim-gazebo.Dockerfile -t cosim-sim-gazebo .
#
# Another simulator gets its own Dockerfile and an entry in orchestrator/simulators.py (docs/CONTAINERS.md).
ARG PX4_RUNTIME_IMAGE=cosim-px4-runtime:latest
FROM ${PX4_RUNTIME_IMAGE}

RUN apt-get update && apt-get install -y --no-install-recommends iproute2 python3-yaml \
    && rm -rf /var/lib/apt/lists/*

# PX4's Gazebo models and server configuration (what px4/fetch_gazebo_models.sh puts in ~/.simulation-gazebo).
COPY px4/fetch_gazebo_models.sh /tmp/fetch_gazebo_models.sh
RUN HOME=/opt/sim-home bash /tmp/fetch_gazebo_models.sh \
    && mv /opt/sim-home/.simulation-gazebo /opt/simulation-gazebo && rm -rf /opt/sim-home /tmp/fetch_gazebo_models.sh

COPY worlds/ /opt/cosim/worlds/
COPY adapters/ /opt/cosim/adapters/
COPY orchestrator/ /opt/cosim/orchestrator/
COPY docker/entrypoints/sim-entrypoint.sh /opt/cosim/entrypoint.sh
ENTRYPOINT ["/opt/cosim/entrypoint.sh"]
