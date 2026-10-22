# syntax=docker/dockerfile:1
#
# The network-emulator container: ns-3 3.48 (with the local patch) plus the wifi_adhoc_tap scenario.
# Build from the repository root (the compose generator does this):
#   docker build -f docker/ns3.Dockerfile -t cosim-ns3 .
#
# Layers are ordered so that editing a scenario rebuilds only the scenario, not ns-3 itself.
FROM ubuntu:24.04 AS build
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake curl ca-certificates bzip2 patch python3 pkg-config libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

ARG NS3_VERSION=3.48
WORKDIR /opt/ns3-build
RUN curl -sSL -O "https://www.nsnam.org/releases/ns-allinone-${NS3_VERSION}.tar.bz2" \
    && tar xjf "ns-allinone-${NS3_VERSION}.tar.bz2" && rm "ns-allinone-${NS3_VERSION}.tar.bz2"
WORKDIR /opt/ns3-build/ns-allinone-${NS3_VERSION}/ns-${NS3_VERSION}

COPY ns3/patches/ /tmp/patches/
RUN for p in /tmp/patches/*.patch; do patch -p1 -N < "$p"; done

# Only the modules the scenarios use (ns-3 pulls in their dependencies). Same optimized profile as ns3/build_ns3.sh.
RUN ./ns3 configure -d optimized --disable-sudo --out="$PWD/build" -G "Unix Makefiles" \
        --enable-modules="core;network;mobility;wifi;tap-bridge;propagation" \
    && ./ns3 build

COPY ns3/scenarios/ /tmp/scenarios/
RUN for d in /tmp/scenarios/*/; do cp -r "${d%/}" "scratch/$(basename "$d")"; done \
    && ./ns3 build scratch/wifi_adhoc_tap/wifi_adhoc_tap

FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        iproute2 nftables iputils-ping tcpdump python3 python3-yaml libyaml-cpp0.8 \
    && rm -rf /var/lib/apt/lists/*
ARG NS3_VERSION=3.48
ENV NS3_ROOT=/opt/ns3-build/ns-allinone-${NS3_VERSION}/ns-${NS3_VERSION}
# Same absolute path as in the build stage: ns-3 binaries carry an RPATH to <root>/build/lib.
COPY --from=build ${NS3_ROOT}/build/lib ${NS3_ROOT}/build/lib
COPY --from=build ${NS3_ROOT}/build/scratch/wifi_adhoc_tap ${NS3_ROOT}/build/scratch/wifi_adhoc_tap
# TapBridge starts this helper (found relative to the ns-3 binary) to open the TAP devices.
COPY --from=build ${NS3_ROOT}/build/src/tap-bridge ${NS3_ROOT}/build/src/tap-bridge
# One stable name per scenario (the technology registry, orchestrator/technologies.py, refers to scenarios by this name).
RUN mkdir -p /opt/cosim/scenarios \
    && ln -s ${NS3_ROOT}/build/scratch/wifi_adhoc_tap/ns${NS3_VERSION}-wifi_adhoc_tap-optimized /opt/cosim/scenarios/wifi_adhoc_tap
COPY orchestrator/ /opt/cosim/orchestrator/
COPY docker/entrypoints/ns3-entrypoint.sh /opt/cosim/entrypoint.sh
ENTRYPOINT ["/opt/cosim/entrypoint.sh"]
