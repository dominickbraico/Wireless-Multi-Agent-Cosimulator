# ns3/

Builds the pinned ns-3 release (3.48) with the local patch, and holds the ns-3 programs that emulate the radio channel.

## Contents

| Path | Purpose |
|---|---|
| `build_ns3.sh` | Fetches ns-3 3.48, applies the patches and builds ns-3 and the scenarios |
| `patches/` | Local patches against ns-3, described in [patches/README.md](patches/README.md) |
| `scenarios/` | The project's ns-3 programs; the build script links them into ns-3's `scratch/` directory |

## Usage

Build ns-3 and the scenarios for the native deployment. The build needs `libyaml-cpp-dev`:

```bash
ns3/build_ns3.sh
```

The Docker deployment builds the same scenarios in `docker/ns3.Dockerfile`. See
[../docs/CONTAINERS.md](../docs/CONTAINERS.md).
