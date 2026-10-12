# syntax=docker/dockerfile:1
#
# Build recipe for the PX4 runtime used by the flight experiments. It builds, at pinned versions:
#   - PX4 SITL (with the gz_x500 airframe patched so it can arm without a ground station)
#   - the Micro XRCE-DDS Agent
#   - the px4_msgs source (rebuilt natively by tools/setup_px4_runtime.sh)
# The image is not run directly. tools/setup_px4_runtime.sh copies these artifacts out so PX4 runs natively
# inside the per-agent network namespaces. Gazebo is neither built nor run here; it runs on the host.
#
#   docker build -t cosim-px4-runtime px4/
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    ROS2_WS=/opt/ros2_ws \
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    PIP_IGNORE_INSTALLED=1

# Version pins. Re-pin the SHAs deliberately instead of tracking branches, so builds stay reproducible.
ARG PX4_AUTOPILOT_REF=2fc441493eb887199c279e2eff990f3f4cd0deef
ARG PX4_MSGS_REF=72fcfaa2750306c9fde8e30e14b5a6d67eb3264f
ARG MICRO_XRCE_DDS_AGENT_TAG=v2.4.3

RUN apt-get update && apt-get install -y --no-install-recommends \
        git \
        wget \
        curl \
        build-essential \
        cmake \
        ninja-build \
        python3-pip \
        sudo \
    && rm -rf /var/lib/apt/lists/*

# PX4-Autopilot: fetch and install its build dependencies. ubuntu.sh includes the Gazebo Harmonic development
# libraries that the SITL Gazebo bridge links against at build time. --no-nuttx skips the embedded toolchain
# because only SITL is built.
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive /opt/PX4-Autopilot \
    && cd /opt/PX4-Autopilot \
    && git checkout ${PX4_AUTOPILOT_REF} \
    && git submodule update --init --recursive \
    && RUNS_IN_DOCKER=true bash ./Tools/setup/ubuntu.sh --no-nuttx

# The pinned commit's gz_x500 airframe defaults to NAV_DLL_ACT=2, which needs an active ground-station or RC link
# to arm. The experiments arm through ROS 2 offboard control with no ground station, so apply the same two parameter
# defaults that upstream later adopted (PX4/PX4-Autopilot#28721). Arming is rejected without this.
RUN sed -i 's/^param set-default NAV_DLL_ACT 2$/param set-default NAV_DLL_ACT 0\nparam set-default CBRK_SUPPLY_CHK 894281/' \
        /opt/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500 \
    && grep -q "NAV_DLL_ACT 0" /opt/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500

# Build the SITL binary and the Gazebo bridge plugin.
RUN cd /opt/PX4-Autopilot && make px4_sitl_default

# Micro XRCE-DDS Agent (bridges PX4's internal uORB bus to ROS 2 DDS).
RUN git clone -b ${MICRO_XRCE_DDS_AGENT_TAG} \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /opt/Micro-XRCE-DDS-Agent \
    && cd /opt/Micro-XRCE-DDS-Agent \
    && mkdir build && cd build \
    && cmake .. \
    && make -j"$(nproc)" \
    && make install \
    && ldconfig /usr/local/lib/

# px4_msgs source at the pinned commit (message definitions must match the PX4 build).
RUN mkdir -p ${ROS2_WS}/src && cd ${ROS2_WS}/src \
    && git clone https://github.com/PX4/px4_msgs.git \
    && cd px4_msgs \
    && git checkout ${PX4_MSGS_REF}
