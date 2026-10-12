#!/usr/bin/env bash
# Download PX4's Gazebo models and server configuration into ~/.simulation-gazebo (needed by the flight experiments and
# the Gazebo GUI demo). Does nothing if they are already present. Does not start Gazebo.
#   px4/fetch_gazebo_models.sh
set -euo pipefail

if [ -d "${HOME}/.simulation-gazebo/models" ] && [ -f "${HOME}/.simulation-gazebo/server.config" ]; then
    echo "PX4 Gazebo models already present in ${HOME}/.simulation-gazebo"
    exit 0
fi

work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
echo "Fetching PX4's simulation-gazebo helper..."
wget -q -O "${work}/simulation-gazebo" https://raw.githubusercontent.com/PX4/PX4-gazebo-models/main/simulation-gazebo
(cd "$work" && python3 simulation-gazebo --dryrun)

[ -d "${HOME}/.simulation-gazebo/models" ] || { echo "Download failed: ${HOME}/.simulation-gazebo/models not created" >&2; exit 1; }
echo "PX4 Gazebo models installed in ${HOME}/.simulation-gazebo"
