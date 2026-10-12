#!/usr/bin/env bash
# Fetch, patch and build ns-3 3.48 for the cosimulator. No sudo required.
#   ns3/build_ns3.sh [install_dir]   (default: ~/dev/ns-3-build)
set -eu
# under plain sudo HOME may be /root; use the invoking user's home so tool paths resolve
[ -n "${SUDO_USER:-}" ] && HOME="$(getent passwd "$SUDO_USER" | cut -d: -f6)"
HERE="$(cd "$(dirname "$0")" && pwd)"
DEST="${1:-$HOME/dev/ns-3-build}"
VER=3.48
mkdir -p "$DEST"; cd "$DEST"
[ -d "ns-allinone-$VER" ] || { curl -sSL -O "https://www.nsnam.org/releases/ns-allinone-$VER.tar.bz2"; tar xjf "ns-allinone-$VER.tar.bz2"; }
cd "ns-allinone-$VER/ns-$VER"
for p in "$HERE"/patches/*.patch; do
  patch -p1 -N --dry-run -s < "$p" >/dev/null 2>&1 && patch -p1 -N < "$p" || echo "skip (already applied): $p"
done
# Our scenarios live in this repo; link them into ns-3's scratch/ so they build with it.
for d in "$HERE"/scenarios/*/; do ln -sfn "${d%/}" "scratch/$(basename "$d")"; done
./ns3 configure -d optimized --disable-sudo --out="$PWD/build" -G "Unix Makefiles"
./ns3 build tap-wifi-virtual-machine scratch/wifi_adhoc_tap/wifi_adhoc_tap
