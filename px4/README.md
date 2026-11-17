# px4/

Builds the PX4 runtime and fetches the Gazebo models that the flight experiments use. The network-only tests need
nothing from this directory.

## Contents

| File | Purpose |
|---|---|
| `Dockerfile` | Builds PX4 SITL, the Micro XRCE-DDS Agent and the `px4_msgs` source at pinned versions |
| `fetch_gazebo_models.sh` | Downloads PX4's Gazebo models and server configuration into `~/.simulation-gazebo` |

The pinned versions are the `ARG` lines at the top of `Dockerfile`.

## Usage

Build the image. This compiles PX4 and takes a long time:

```bash
docker build -t cosim-px4-runtime px4/
```

For the native deployment, also fetch the models and extract the runtime into `~/dev/px4-runtime`:

```bash
px4/fetch_gazebo_models.sh
tools/setup_px4_runtime.sh
```

The Docker deployment builds its agent and simulator images on top of `cosim-px4-runtime`, so build it first. See
[../docs/CONTAINERS.md](../docs/CONTAINERS.md). In the native deployment the image is only a build environment: PX4
runs natively inside the per-agent network namespaces.
