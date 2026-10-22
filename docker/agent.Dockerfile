# syntax=docker/dockerfile:1
#
# One image for every agent container: PX4 SITL, the Micro XRCE-DDS Agent, ROS 2 Jazzy and this repository's ROS 2
# packages. Build from the repository root:
#   docker build -f docker/agent.Dockerfile -t cosim-agent .
#
# Built on the PX4 runtime image (px4/Dockerfile, pinned PX4 / XRCE agent / px4_msgs) so the PX4 binary the experiments
# fly is the one the native setup extracts, and the Gazebo client libraries PX4's gz bridge links against are present.
# The base is large (it still holds the PX4 build tree) but shares its layers with the simulator image.
ARG PX4_RUNTIME_IMAGE=cosim-px4-runtime:latest
FROM ${PX4_RUNTIME_IMAGE}

RUN apt-get update && apt-get install -y --no-install-recommends \
        iproute2 nftables iputils-ping tcpdump python3-yaml python3-colcon-common-extensions \
        ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 workspace: the px4_msgs source pinned in the base image, plus this repository's packages.
COPY ros2_ws/src /opt/cosim_ws/src
RUN cp -r /opt/ros2_ws/src/px4_msgs /opt/cosim_ws/src/px4_msgs \
    && . /opt/ros/jazzy/setup.sh && cd /opt/cosim_ws && colcon build --event-handlers console_direct- \
    && rm -rf /opt/cosim_ws/build /opt/cosim_ws/log

COPY config/dds/ /opt/cosim/config/dds/
COPY orchestrator/ /opt/cosim/orchestrator/
COPY docker/entrypoints/agent-entrypoint.sh /opt/cosim/entrypoint.sh
ENTRYPOINT ["/opt/cosim/entrypoint.sh"]
