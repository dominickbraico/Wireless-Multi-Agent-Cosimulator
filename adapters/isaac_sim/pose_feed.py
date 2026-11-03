"""Pose feed from Isaac Sim to ns-3: UDP/JSON datagrams in the contract of docs/CONTAINERS.md (the same as adapters/gazebo/pose_bridge.py).

    {"seq": n, "t_sim": s, "t_wall": s, "agents": {"agent1": [x, y, z], ...}}     antenna position, metres, East-North-Up, same origin as ns-3

PoseSender runs in its own thread inside the Isaac Sim process, so the feed keeps going at `rate_hz` in wall-clock time even when the simulation loop
is blocked (Pegasus waits for PX4 in lockstep): the last pose is repeated, `t_wall` advances, and ns-3 reports how stale the pose is from it.
The simulation loop calls update() after each physics step. Pure Python: no Isaac Sim imports, so it is tested outside the simulator (tests/isaac_pose_feed.sh).
"""
import json
import socket
import threading
import time


def encode(seq, t_sim, agents):
    """Wire encoding (JSON), kept behind one function as in the Gazebo bridge."""
    return json.dumps({"seq": seq, "t_sim": round(t_sim, 6), "t_wall": round(time.time(), 6), "agents": agents},
                      separators=(",", ":")).encode()


class PoseSender:
    def __init__(self, addr, rate_hz, antenna_offsets):
        """addr: (host, port) of the ns-3 pose listener. antenna_offsets: {agent: [dx, dy, dz]} added to each vehicle position."""
        self.addr = addr
        self.period = 1.0 / float(rate_hz)
        self.offsets = antenna_offsets
        self._latest = {}                    # agent -> [x, y, z] of the vehicle (before the antenna offset)
        self._t_sim = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def update(self, agent, position, t_sim):
        """Called by the simulation loop; cheap and thread-safe."""
        with self._lock:
            self._latest[agent] = [float(position[0]), float(position[1]), float(position[2])]
            self._t_sim = float(t_sim)

    def start(self):
        self._thread = threading.Thread(target=self._run, name="pose-feed", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _run(self):
        seq, nxt = 0, time.time()
        while not self._stop.is_set():
            nxt += self.period
            with self._lock:
                agents = {n: [round(p[i] + self.offsets[n][i], 6) for i in range(3)] for n, p in self._latest.items()}
                t_sim = self._t_sim
            if agents:                       # nothing is sent before the first update, as in the Gazebo bridge
                self._sock.sendto(encode(seq, t_sim, agents), self.addr)
                seq += 1
            self._stop.wait(max(0.0, nxt - time.time()))
