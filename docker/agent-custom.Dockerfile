# syntax=docker/dockerfile:1
#
# Agent image for a `scenario.type: custom` experiment: the stock agent image plus the user's own colcon workspace
# (scenario.custom.workspace in the experiment YAML; docs/CUSTOM_CONTROLLERS.md). The build context is that workspace:
#   docker build -f docker/agent-custom.Dockerfile -t cosim-agent-custom-<experiment name> <workspace>
# `python3 orchestrator/cosim.py build --config <yaml>` does this (orchestrator/compose_gen.py names the tag).
#
# The workspace is built as an overlay on /opt/cosim_ws, so its packages can depend on this repository's packages
# (px4_msgs, px4_offboard_control, ...). The base image has a C++ toolchain, CMake, colcon and an initialised rosdep
# (checked 2026-11-17 on cosim-agent built from px4/Dockerfile's pinned versions).
ARG AGENT_IMAGE=cosim-agent:latest
FROM ${AGENT_IMAGE}

COPY src /opt/user_ws/src
# rosdep installs the system dependencies the packages declare. `rosdep check` runs offline against the cache in the
# base image; only when it reports something missing does the build go to the network (apt-get update, rosdep install).
# Keys rosdep does not know (for example this repository's own packages) are skipped (-r); a truly missing dependency
# then fails the colcon build below with its name.
RUN . /opt/ros/jazzy/setup.sh && . /opt/cosim_ws/install/setup.sh && cd /opt/user_ws \
    && if rosdep check --from-paths src --ignore-src --rosdistro jazzy; then echo "agent-custom: all system dependencies present"; \
       else (apt-get -o Acquire::Retries=1 -o Acquire::http::Timeout=20 update && rosdep update --rosdistro jazzy \
             && rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy) \
            || echo "agent-custom: rosdep could not resolve every dependency; continuing with the build"; fi \
    && colcon build --event-handlers console_direct- \
    && rm -rf build log /var/lib/apt/lists/*
