#!/usr/bin/env python3
"""Gazebo Fuel models for worlds that reference them (e.g. baylands), made available to the offline simulator container.

The compose stack has no internet access (its networks are internal). A world whose <include><uri> points at
https://fuel.gazebosim.org/... would therefore fail to load. Instead the models are downloaded once, on the host side, into a
persistent cache that the simulator container mounts at ~/.gz/fuel:

    ensure(world_sdf, ...)    if the world needs Fuel models and the cache has not been filled for this version of the file,
                              run Gazebo once in a throw-away container WITH internet and let it resolve everything
                              (including models that other models depend on), then record a marker.
    fuel.py fetch <world>     the same, from the command line

Cache location: $COSIM_FUEL_CACHE, default ~/.cache/cosim/fuel. Set COSIM_NO_FETCH=1 to forbid downloads (offline runs).
Reproducibility: Fuel serves the latest version of a model that a URI without a version names, so a fetch made later can
differ from an earlier one. `describe()` records which versions were cached; cosim.py writes that to <log dir>/fuel_models.txt.
"""
import hashlib
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
SIM_IMAGE = "cosim-sim-gazebo:latest"
CACHE_ENV = "COSIM_FUEL_CACHE"
URI_RE = re.compile(r"<uri>\s*(https?://[^<]+?)\s*</uri>")


def cache_dir():
    return os.path.abspath(os.path.expanduser(os.environ.get(CACHE_ENV, "~/.cache/cosim/fuel")))


def world_path(world):
    return os.path.join(ROOT, "worlds", f"{world}.sdf")


def remote_uris(sdf_path):
    """http(s) model URIs written directly in the world file (models that those models depend on are not listed)."""
    return URI_RE.findall(open(sdf_path).read())


def _digest(sdf_path):
    return hashlib.sha256(open(sdf_path, "rb").read()).hexdigest()


def _marker(world):
    return os.path.join(cache_dir(), ".cosim", f"{world}.sha256")


def is_ready(world):
    sdf = world_path(world)
    if not remote_uris(sdf):
        return True
    try:
        return open(_marker(world)).read().strip() == _digest(sdf)
    except OSError:
        return False


def fetch(world, timeout_s=1800):
    """Download every model the world needs into the cache. Returns True on success."""
    sdf = world_path(world)
    cache = cache_dir()
    os.makedirs(os.path.join(cache, ".cosim"), exist_ok=True)
    # run as the invoking user so the cache is theirs, not root's
    script = f"""
export PATH=/usr/bin:$PATH GZ_CONFIG_PATH=/usr/share/gz GZ_PARTITION=cosim_fetch
export GZ_SIM_RESOURCE_PATH=/opt/simulation-gazebo/models:/opt/simulation-gazebo/worlds:/opt/cosim/worlds/models
export GZ_SIM_SERVER_CONFIG_PATH=/opt/simulation-gazebo/server.config
/usr/bin/gz sim -s -r --headless-rendering /opt/cosim/worlds/{world}.sdf > /tmp/gz_fetch.log 2>&1 &
pid=$!
ok=0
for i in $(seq 1 {timeout_s // 3}); do
  if gz topic -l 2>/dev/null | grep -q /world/{world}/stats; then ok=1; break; fi
  kill -0 $pid 2>/dev/null || break
  sleep 3
done
kill $pid 2>/dev/null; wait $pid 2>/dev/null
[ $ok = 1 ] || {{ tail -n 25 /tmp/gz_fetch.log; exit 1; }}
"""
    print(f"fuel: downloading the Fuel models that world '{world}' needs into {cache} "
          f"(first use only; large worlds are several hundred MB)", flush=True)
    cmd = ["docker", "run", "--rm", "--user", f"{os.getuid()}:{os.getgid()}", "-e", "HOME=/tmp/fuelhome",
           "-v", f"{cache}:/tmp/fuelhome/.gz/fuel", "-v", f"{os.path.dirname(sdf)}:/opt/cosim/worlds:ro",
           "--entrypoint", "bash", SIM_IMAGE, "-c", script]
    if subprocess.run(cmd).returncode != 0:
        print(f"fuel: the world did not load; models may be missing from {cache}", file=sys.stderr)
        return False
    open(_marker(world), "w").write(_digest(sdf) + "\n")
    return True


def ensure(world):
    """Make the world's Fuel models available offline. Raises SystemExit with the remedy if they cannot be."""
    if is_ready(world):
        return
    if os.environ.get("COSIM_NO_FETCH") == "1":
        raise SystemExit(f"world '{world}' needs Fuel models that are not cached in {cache_dir()} and COSIM_NO_FETCH=1 forbids "
                         f"downloading. Run: python3 orchestrator/fuel.py fetch {world}")
    if not fetch(world):
        raise SystemExit(f"could not fetch the Fuel models for world '{world}' (needs internet and the cosim-sim-gazebo image)")


def describe(world):
    """Lines naming the Fuel models the world references and the versions found in the cache (top-level URIs only)."""
    out = [f"world {world}: {_digest(world_path(world))[:12]} (sha256 of the .sdf)", f"cache {cache_dir()}"]
    for u in remote_uris(world_path(world)):
        m = re.match(r"https?://([^/]+)/[^/]+/([^/]+)/models/(.+)$", u)
        if not m:
            out.append(f"{u}: unrecognised URI form")
            continue
        host, owner, name = m.group(1), m.group(2).lower(), m.group(3).lower()
        d = os.path.join(cache_dir(), host, owner, "models", name)
        vers = sorted(os.listdir(d)) if os.path.isdir(d) else []
        out.append(f"{u}: cached versions {vers or 'none'}")
    return out


def main():
    if len(sys.argv) != 3 or sys.argv[1] not in ("fetch", "status"):
        sys.exit(__doc__)
    w = sys.argv[2]
    if not os.path.isfile(world_path(w)):
        sys.exit(f"no such world: {world_path(w)}")
    if sys.argv[1] == "fetch":
        sys.exit(0 if fetch(w) else 1)
    print("\n".join(describe(w)))
    print("ready" if is_ready(w) else "NOT ready: run fuel.py fetch " + w)


if __name__ == "__main__":
    main()
