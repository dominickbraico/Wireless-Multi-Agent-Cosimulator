#!/usr/bin/env python3
"""Decode batman-adv frames in a capture of an agent's `mesh0` interface (Ethernet pcap, written by tcpdump).

    batman_pcap.py <pcap> [--json]

Prints one line per (outer source MAC, outer destination MAC, inner IPv4 source, inner IPv4 destination, batman TTL) of
batman-adv unicast frames that carry IPv4, with the number of frames, and a count of every frame type. A relay shows up as
the same inner packet arriving from one neighbour and leaving to another with a TTL one lower.

batman-adv compat version 15 (the kernel's batman-adv 2024.0), enum batadv_packettype:
    0x00 iv_ogm (B.A.T.M.A.N. IV originator message)   0x01 bcast   0x02 coded   0x03 elp   0x04 ogm2   0x05 mcast
    0x40 unicast (header: type, version, ttl, ttvn, dest[6], then the inner Ethernet frame)
    0x41 unicast_frag   0x42 unicast_4addr   0x43 icmp   0x44 unicast_tvlv
Only plain unicast (0x40) is decoded further. `--from MAC[,MAC...]` limits the non-batman count to frames sent by those MACs
(a capture on mesh0 also holds frames arriving from the host's Docker bridge, which ns-3's source-MAC guard drops). Reference: net/batman-adv/packet.h in the Linux kernel (include/uapi/linux/batadv_packet.h).
"""
import argparse
import collections
import json
import struct
import sys

ETH_P_BATMAN = 0x4305
TYPES = {0x00: "iv_ogm", 0x01: "bcast", 0x02: "coded", 0x03: "elp", 0x04: "ogm2", 0x05: "mcast",
         0x40: "unicast", 0x41: "unicast_frag", 0x42: "unicast_4addr", 0x43: "icmp", 0x44: "unicast_tvlv"}


def frames(path):
    """Ethernet frames of a classic pcap file (microsecond or nanosecond timestamps, either byte order)."""
    d = open(path, "rb").read()
    magic = d[:4]
    endian = "<" if magic in (b"\xd4\xc3\xb2\xa1", b"\x4d\x3c\xb2\xa1") else ">"
    linktype = struct.unpack(endian + "I", d[20:24])[0]
    if linktype != 1:
        sys.exit(f"batman_pcap.py: {path} is not an Ethernet capture (link type {linktype})")
    off = 24
    while off + 16 <= len(d):
        sec, frac, incl, _ = struct.unpack(endian + "IIII", d[off:off + 16])
        yield sec + frac * 1e-6, d[off + 16:off + 16 + incl]
        off += 16 + incl


def decode(path, senders=None):
    """-> (frame kinds, unicast IPv4 flows). With `senders` (set of MACs), non-batman frames from other sources are not counted."""
    kinds = collections.Counter()
    flows = collections.Counter()
    for _, f in frames(path):
        if len(f) < 14:
            continue
        et = struct.unpack(">H", f[12:14])[0]
        if et != ETH_P_BATMAN:
            if senders is None or f[6:12].hex(":") in senders:
                kinds[f"non-batman ethertype 0x{et:04x}"] += 1
            continue
        ptype = f[14] if len(f) > 14 else -1
        kinds[TYPES.get(ptype, f"batman type 0x{ptype:02x}")] += 1
        if ptype != 0x40 or len(f) < 24 + 34:
            continue
        ttl, inner = f[16], f[24:]
        if struct.unpack(">H", inner[12:14])[0] != 0x0800:
            continue
        ip = inner[14:34]
        flows[(f[6:12].hex(":"), f[0:6].hex(":"), ".".join(map(str, ip[12:16])), ".".join(map(str, ip[16:20])), ttl)] += 1
    return kinds, flows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("pcap")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--from", dest="senders", default=None, help="comma-separated MACs: count non-batman frames from these senders only")
    a = ap.parse_args()
    kinds, flows = decode(a.pcap, set(a.senders.lower().split(",")) if a.senders else None)
    if a.json:
        print(json.dumps({"kinds": kinds, "flows": [dict(zip(("src", "dst", "ip_src", "ip_dst", "ttl"), k), frames=v)
                                                    for k, v in flows.items()]}))
        return
    print("frames: " + ", ".join(f"{k} {v}" for k, v in sorted(kinds.items())))
    for (src, dst, isrc, idst, ttl), n in sorted(flows.items(), key=lambda kv: (kv[0][2], kv[0][3], -kv[0][4])):
        print(f"{src} -> {dst}  carries IPv4 {isrc} -> {idst}  ttl {ttl}: {n} frames")


if __name__ == "__main__":
    main()
