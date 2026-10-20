# px4/

Everything needed to obtain the PX4 runtime and Gazebo models used by the flight experiments. Nothing here is needed for the
network-only tests.

| File | Purpose |
|---|---|
| `Dockerfile` | Builds PX4 SITL, the Micro XRCE-DDS Agent and the `px4_msgs` source at pinned versions |
| `fetch_gazebo_models.sh` | Downloads PX4's Gazebo models and server configuration into `~/.simulation-gazebo` |

```
docker build -t cosim-px4-runtime px4/        # long build: compiles PX4
px4/fetch_gazebo_models.sh
tools/setup_px4_runtime.sh                    # extracts the runtime into ~/dev/px4-runtime
```

The Docker deployment ([../docs/CONTAINERS.md](../docs/CONTAINERS.md)) builds its agent and simulator images on top of this image (`docker/agent.Dockerfile`,
`docker/sim-gazebo.Dockerfile`), so build it first. For the native setup the image is only a build environment. `tools/setup_px4_runtime.sh` copies PX4, the XRCE agent and `px4_msgs` out of it so PX4 runs natively
inside the per-agent network namespaces. The pinned versions are the `ARG` lines at the top of the Dockerfile.
