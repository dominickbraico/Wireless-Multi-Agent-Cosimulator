#!/usr/bin/env python3
"""Stand-in for a physics simulator's MAVLink server, to test PX4's side of the link without the simulator.

    mavlink_sim_stub.py <port> <seconds>

PX4's simulator_mavlink module connects out to the simulator over TCP (port 4560 + PX4 instance; PX4_SIM_HOST_ADDR names the host).
This stub plays the simulator's end: it accepts one connection, counts what PX4 sends, and prints
    stub: PX4 connected from <ip>
    stub: received <n> bytes
It does not parse MAVLink and never answers, so PX4 will not leave its wait for sensor data; it only proves the connection.
Used by tests/px4_mavlink_link.sh (Isaac Sim spike, stage A). Exits 1 if no connection arrives within <seconds>.
"""
import socket
import sys
import time


def main(port: int, seconds: float) -> int:
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", port))
    srv.listen(1)
    deadline = time.time() + seconds
    srv.settimeout(max(0.1, deadline - time.time()))
    try:
        conn, addr = srv.accept()
    except socket.timeout:
        print(f"stub: no connection within {seconds:g} s", flush=True)
        return 1
    print(f"stub: PX4 connected from {addr[0]}", flush=True)
    total = 0
    while time.time() < deadline:
        conn.settimeout(max(0.1, min(1.0, deadline - time.time())))
        try:
            data = conn.recv(65536)
        except socket.timeout:
            continue
        if not data:                       # PX4 closed the connection
            break
        total += len(data)
        print(f"stub: received {total} bytes", flush=True)     # running total, so a reader need not wait for the end
    print(f"stub: received {total} bytes", flush=True)
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    sys.exit(main(int(sys.argv[1]), float(sys.argv[2])))
