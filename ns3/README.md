# ns3/

- `build_ns3.sh` fetches, patches (see `patches/README.md`) and builds ns-3 3.48.
- `patches/` local patches against ns-3.
- `scenarios/` our ns-3 programs (symlinked into scratch/ by the build script). Needs libyaml-cpp-dev.
- The Docker deployment builds the same scenarios in `docker/ns3.Dockerfile` ([../docs/CONTAINERS.md](../docs/CONTAINERS.md)).
