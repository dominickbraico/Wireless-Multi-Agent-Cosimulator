"""Sends camera frames from the Isaac Sim process to an agent container's camera_sink (orchestrator/camera_sink.py) over TCP.

Frames are handed over by the simulation loop with push() and sent by a thread, so a slow or absent receiver never blocks the simulation: at most 2 frames are
queued and the oldest is dropped when the queue is full (the count is kept in `dropped`). The thread connects with retries, because the agent container's
receiver may start later than the simulator. Pure Python (no Isaac Sim imports).
"""
import collections
import socket
import sys
import threading
import time

import camera_sink                                           # orchestrator/ is on PYTHONPATH in the simulator container


class FrameStreamer:
    def __init__(self, addr):
        self.addr = addr
        self.dropped = 0
        self.sent = 0
        self._q = collections.deque(maxlen=2)
        self._cv = threading.Condition()
        self._stop = False
        self._thread = threading.Thread(target=self._run, name=f"camera-{addr[1]}", daemon=True)

    def start(self):
        self._thread.start()

    def push(self, seq, t_sim, width, height, data):
        with self._cv:
            if len(self._q) == self._q.maxlen:
                self.dropped += 1
            self._q.append(camera_sink.pack_frame(seq, t_sim, width, height, data))
            self._cv.notify()

    def stop(self):
        with self._cv:
            self._stop = True
            self._cv.notify()

    def _connect(self):
        while not self._stop:
            try:
                s = socket.create_connection(self.addr, timeout=2.0)
                s.settimeout(5.0)
                return s
            except OSError:
                time.sleep(1.0)
        return None

    def _run(self):
        sock = None
        while not self._stop:
            with self._cv:
                while not self._q and not self._stop:
                    self._cv.wait()
                msg = self._q.popleft() if self._q else None
            if msg is None:
                continue
            for _ in range(2):
                if sock is None:
                    sock = self._connect()
                    if sock is None:
                        return
                try:
                    sock.sendall(msg)
                    self.sent += 1
                    break
                except OSError:
                    print("camera_stream: connection lost, reconnecting", file=sys.stderr, flush=True)
                    sock.close()
                    sock = None
