#!/usr/bin/env python3
"""Gazebo pose bridge: streams agent antenna positions to the ns-3 pose listener.

Subscribes to /world/<world>/<pose_topic> (gz Pose_V), keeps the latest pose per
configured agent, applies the antenna offset, and sends a UDP datagram at a fixed
rate (re-sending the last known pose when nothing changed, so ns-3 always has a
fresh heartbeat). Also records Gazebo's real-time factor from /world/<w>/stats.

Wire format (JSON, see adapters/README.md; keep it behind encode()):
  {"seq": n, "t_sim": s, "t_wall": s, "agents": {"agent1": [x,y,z], ...}}
Frame: Gazebo world ENU meters == ns-3 Vector(x,y,z), same origin.

    pose_bridge.py --config <experiment.yaml> [--log-dir DIR] [--duration S]
"""
import argparse
import json
import os
import socket
import threading
import time

import yaml
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.transport13 import Node


def encode(seq, t_sim, agents):
    """Swappable wire encoding (JSON now; protobuf/ZeroMQ later)."""
    return json.dumps({"seq": seq, "t_sim": round(t_sim, 6), "t_wall": round(time.time(), 6),
                       "agents": agents}, separators=(",", ":")).encode()


class PoseBridge:
    def __init__(self, cfg, log_dir, world=None, px4_models=False):
        sim = cfg["simulator"]
        self.world = world or sim["world_name"]
        self.topic = f"/world/{self.world}/{sim.get('pose_topic', 'pose/info')}"
        self.stats_topic = f"/world/{self.world}/stats"
        # Gazebo model name -> agent name. Box worlds use the agent names; PX4 worlds spawn x500_<instance>.
        key = "px4_gz_model" if px4_models else "name"
        self.model_to_agent = {a[key]: a["name"] for a in cfg["agents"]}
        self.offsets = {a["name"]: a["antenna_offset"] for a in cfg["agents"]}
        pose = cfg["network"]["pose"]
        self.addr = (pose["listen_addr"], int(pose["port"]))
        self.period = 1.0 / float(pose["rate_hz"])
        self.latest = {}          # name -> [x,y,z] (raw model position)
        self.t_sim = 0.0
        self.lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rtf = []
        self.log_dir = log_dir
        os.makedirs(log_dir, exist_ok=True)
        self.rtf_csv = open(os.path.join(log_dir, "rtf.csv"), "w", buffering=1)
        self.rtf_csv.write("t_wall_s,t_sim_s,rtf\n")
        self.node = Node()
        if not self.node.subscribe(Pose_V, self.topic, self.on_pose):
            raise SystemExit(f"cannot subscribe to {self.topic}")
        self.node.subscribe(WorldStatistics, self.stats_topic, self.on_stats)

    def on_pose(self, msg):
        with self.lock:
            for p in msg.pose:
                if p.name in self.model_to_agent:
                    self.latest[self.model_to_agent[p.name]] = [p.position.x, p.position.y, p.position.z]
            self.t_sim = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9

    def on_stats(self, msg):
        t = msg.sim_time.sec + msg.sim_time.nsec * 1e-9
        self.rtf_csv.write(f"{time.time():.3f},{t:.3f},{msg.real_time_factor:.4f}\n")
        self.rtf.append(msg.real_time_factor)

    def run(self, duration):
        seq, end = 0, (time.time() + duration if duration else None)
        nxt = time.time()
        while end is None or time.time() < end:
            nxt += self.period
            with self.lock:
                agents = {n: [p[i] + self.offsets[n][i] for i in range(3)]
                          for n, p in self.latest.items()}
                t_sim = self.t_sim
            if agents:
                self.sock.sendto(encode(seq, t_sim, agents), self.addr)
                seq += 1
            time.sleep(max(0.0, nxt - time.time()))
        return seq


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--log-dir", default="logs/pose_bridge")
    ap.add_argument("--duration", type=float, default=0, help="seconds; 0 = run until killed")
    ap.add_argument("--world", default=None, help="override simulator.world_name (e.g. px4_flat)")
    ap.add_argument("--px4-models", action="store_true", help="match Gazebo models by agents[].px4_gz_model")
    a = ap.parse_args()
    with open(a.config) as f:
        cfg = yaml.safe_load(f)
    b = PoseBridge(cfg, a.log_dir, a.world, a.px4_models)
    n = b.run(a.duration)
    if b.rtf:
        print(f"pose_bridge sent {n} datagrams; RTF mean {sum(b.rtf)/len(b.rtf):.3f} min {min(b.rtf):.3f}")


if __name__ == "__main__":
    main()
