# syntax=docker/dockerfile:1
#
# The physics-simulator container for Isaac Sim (spike, experimental): Isaac Sim 5.1.0 headless with Pegasus Simulator v5.1.0 (multirotors only).
# Build from the repository root (the base image comes from NVIDIA's registry; `docker login nvcr.io` with an NGC API key, or pull it first):
#   docker build -f docker/sim-isaac.Dockerfile -t cosim-sim-isaac .
# Running it needs the NVIDIA Container Toolkit and COSIM_ACCEPT_ISAAC_EULA=Y (orchestrator/simulators.py; docs/CONTAINERS.md, "Isaac Sim").
FROM nvcr.io/nvidia/isaac-sim:5.1.0

# Root, because the base image's user (isaac-sim, uid 1234) cannot write the bind-mounted /logs, and /isaac-sim is readable by that user only.
USER root

# Pegasus at a pinned commit (the v5.1.0 tag), used from PYTHONPATH without `pip install -e`: its setup.py edits Isaac Sim's kit app files. The container
# network is internal at run time, so everything is fetched here. pymavlink is its only dependency that Isaac Sim's Python lacks (checked on 2026-11-03).
ADD --keep-git-dir=false https://github.com/PegasusSimulator/PegasusSimulator.git#644da37e9d5268e5f9a34e78bdcfd57a8bab82b4 /opt/pegasus
RUN /isaac-sim/python.sh -m pip install --no-cache-dir --target /opt/pylib pymavlink

COPY adapters/ /opt/cosim/adapters/
COPY orchestrator/ /opt/cosim/orchestrator/
COPY docker/entrypoints/isaac-entrypoint.sh /opt/cosim/entrypoint.sh
ENTRYPOINT ["/opt/cosim/entrypoint.sh"]
