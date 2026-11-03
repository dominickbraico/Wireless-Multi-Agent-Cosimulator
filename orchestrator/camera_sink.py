#!/usr/bin/env python3
"""Camera frame stream between the simulator container and an agent container (Isaac Sim spike, stage F).

    camera_sink.py --port P --csv FILE [--ppm-dir DIR --ppm-every N]          (runs in the agent container; started by agent_main.py when the agent has a `camera:` block)

The Isaac Sim process connects to the agent's management-network address, TCP port PORT_BASE + px4_instance, and sends one message per frame:
    header  struct "<4sIIIId"  magic b"CAM1", seq, width, height, nbytes, t_sim      then  nbytes of raw RGB (8 bit, row-major, width x height x 3)
The stream never touches the emulated radio: it runs over the simulator <-> agent management network, as the PX4 link does. The receiver appends one CSV row per
frame: t_wall_s (arrival), seq, t_sim_s, width, height, nbytes, size_ok (nbytes == width x height x 3), std (standard deviation of every 97th byte; 0 for a constant image).
This is a transport for the spike, not a ROS 2 topic: a lab member's node would need a small publisher on top (not implemented).
"""
import argparse
import os
import socket
import statistics
import struct
import threading
import time

PORT_BASE = 5700
HEADER = struct.Struct("<4sIIIId")
MAGIC = b"CAM1"
MAX_FRAME_BYTES = 64 * 1024 * 1024        # a header claiming more is treated as garbage and the connection is dropped


def pack_frame(seq, t_sim, width, height, data):
    return HEADER.pack(MAGIC, seq, width, height, len(data), t_sim) + data


def _recv_exact(conn, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(min(1 << 20, n - len(buf)))
        if not chunk:
            return None
        buf += chunk
    return bytes(buf)


class Sink:
    def __init__(self, addr, csv_path, ppm_dir=None, ppm_every=0):
        self.ppm_dir, self.ppm_every = ppm_dir, int(ppm_every)       # optional debugging aid: every Nth frame also saved as a binary PPM
        if self.ppm_dir and self.ppm_every:
            os.makedirs(self.ppm_dir, exist_ok=True)
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(addr)
        self.srv.listen(1)
        self.srv.settimeout(0.2)
        self.port = self.srv.getsockname()[1]
        self._stop = threading.Event()
        self.csv = open(csv_path, "w", buffering=1)
        self.csv.write("t_wall_s,seq,t_sim_s,width,height,nbytes,size_ok,std\n")

    def stop(self):
        self._stop.set()

    def run(self):
        while not self._stop.is_set():
            try:
                conn, _ = self.srv.accept()
            except socket.timeout:
                continue
            conn.settimeout(1.0)
            try:
                self._serve(conn)
            finally:
                conn.close()

    def _serve(self, conn):
        while not self._stop.is_set():
            try:
                head = _recv_exact(conn, HEADER.size)
                if head is None:
                    return
                magic, seq, w, h, n, t_sim = HEADER.unpack(head)
                if magic != MAGIC or n > MAX_FRAME_BYTES:
                    return
                data = _recv_exact(conn, n)
                if data is None:
                    return
            except socket.timeout:
                continue
            sample = list(data[::97])
            std = statistics.pstdev(sample) if len(sample) > 1 else 0.0
            self.csv.write(f"{time.time():.6f},{seq},{t_sim:.6f},{w},{h},{n},{int(n == w * h * 3)},{std:.3f}\n")
            if self.ppm_dir and self.ppm_every and seq % self.ppm_every == 0 and n == w * h * 3:
                with open(os.path.join(self.ppm_dir, f"frame_{seq:06d}.ppm"), "wb") as f:
                    f.write(f"P6\n{w} {h}\n255\n".encode() + data)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, required=True)
    ap.add_argument("--csv", required=True)
    ap.add_argument("--ppm-dir", default=None, help="directory for saved frames (with --ppm-every)")
    ap.add_argument("--ppm-every", type=int, default=0, help="save every Nth frame as a PPM; 0 = never")
    a = ap.parse_args()
    Sink(("0.0.0.0", a.port), a.csv, a.ppm_dir, a.ppm_every).run()


if __name__ == "__main__":
    main()
