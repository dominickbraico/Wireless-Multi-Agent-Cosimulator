#!/usr/bin/env python3
"""Constant-rate UDP probe with per-packet logging (stands in for / complements iperf3).

  udp_probe.py tx <dst_ip> <port> <rate_bps> <size_B> <duration_s> <out.csv>   # logs seq,t_tx
  udp_probe.py rx <bind_ip> <port> <duration_s> <out.csv>                     # logs seq,t_rx
Wall-clock epoch seconds; same host so timestamps are comparable across namespaces.
"""
import socket
import sys
import time


def tx(dst, port, rate, size, dur, out):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gap = size * 8 / rate
    f = open(out, "w", buffering=1 << 16)
    f.write("seq,t_tx\n")
    t_end = time.time() + dur
    nxt = time.time()
    seq = 0
    while time.time() < t_end:
        t = time.time()
        s.sendto(seq.to_bytes(4, "big") + b"x" * (size - 4), (dst, port))
        f.write(f"{seq},{t:.6f}\n")
        seq += 1
        nxt += gap
        time.sleep(max(0.0, nxt - time.time()))
    f.close()


def rx(bind, port, dur, out):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((bind, port))
    s.settimeout(0.5)
    f = open(out, "w", buffering=1 << 16)
    f.write("seq,t_rx\n")
    t_end = time.time() + dur
    while time.time() < t_end:
        try:
            d, _ = s.recvfrom(4096)
        except socket.timeout:
            continue
        f.write(f"{int.from_bytes(d[:4], 'big')},{time.time():.6f}\n")
    f.close()


if __name__ == "__main__":
    m = sys.argv[1]
    if m == "tx":
        tx(sys.argv[2], int(sys.argv[3]), float(sys.argv[4]), int(sys.argv[5]), float(sys.argv[6]), sys.argv[7])
    else:
        rx(sys.argv[2], int(sys.argv[3]), float(sys.argv[4]), sys.argv[5])
