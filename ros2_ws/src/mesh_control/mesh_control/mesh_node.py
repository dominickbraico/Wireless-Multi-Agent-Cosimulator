"""Mesh heartbeat node: every agent broadcasts its own state on the emulated radio and tracks which neighbours it can hear.

For the mesh-dropout scenario (docs/MESH_DROPOUT.md). Like swarm_control's swarm node it is the ONLY process of an agent on the radio DDS
domain; the vehicle's state comes from the local (loopback-only) domain through the reused `LocalSide`. All-to-all: each agent publishes
`mesh_msgs/PeerState` on /mesh/peer_state and subscribes to everyone else's.

Per neighbour it keeps the last sequence number and the time it was last heard, and writes two CSV files (wall-clock seconds):
    mesh_rx_<agent>.csv      t_rx_s, peer_id, seq, latency_ms, gap          one row per received heartbeat (gap = lost in between)
    mesh_events_<agent>.csv  t_s, agent_id, peer_id, event, silence_s       UP when a neighbour is first heard or heard again,
                                                                            LOST when it has been silent for link_timeout_s
`silence_s` on an UP event after a LOST is how long the neighbour had been silent. LOST is therefore detected `link_timeout_s` after the
last heartbeat, not at the instant the signal fell under the receiver floor.

Parameters (ROS parameters, set by orchestrator/scenario_mesh.py from the YAML):
    agent_id, px4_instance, spawn_xy, agent_name, log_dir, heartbeat_hz, link_timeout_s, qos_reliability, qos_durability, qos_depth
"""
import csv
import os
import sys
import threading
import time

import rclpy
from mesh_msgs.msg import PeerState
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from swarm_control.swarm_node import LocalSide, _cli_params


class MeshNode(Node):
    """Radio side. `state_provider()` -> (pos, vel) in the shared ENU world frame, or None before the vehicle has a valid state."""

    def __init__(self, context, state_provider):
        super().__init__('mesh_node', context=context)
        self.state_provider = state_provider
        self.declare_parameter('agent_id', 1)
        self.declare_parameter('agent_name', 'agent')
        self.declare_parameter('log_dir', '')
        self.declare_parameter('heartbeat_hz', 5.0)
        self.declare_parameter('link_timeout_s', 1.5)
        self.declare_parameter('qos_reliability', 'best_effort')
        self.declare_parameter('qos_durability', 'volatile')
        self.declare_parameter('qos_depth', 10)     # keyless topic, several publishers: the history is shared by all senders (docs/MESH_DROPOUT.md)
        self.agent_id = int(self.get_parameter('agent_id').value)
        name = self.get_parameter('agent_name').value
        self.timeout = float(self.get_parameter('link_timeout_s').value)
        rel = {'best_effort': ReliabilityPolicy.BEST_EFFORT, 'reliable': ReliabilityPolicy.RELIABLE}
        dur = {'volatile': DurabilityPolicy.VOLATILE, 'transient_local': DurabilityPolicy.TRANSIENT_LOCAL}
        r, d = self.get_parameter('qos_reliability').value, self.get_parameter('qos_durability').value
        depth = int(self.get_parameter('qos_depth').value)
        if r not in rel or d not in dur or depth < 1:
            raise ValueError(f"qos: reliability in {list(rel)}, durability in {list(dur)}, depth >= 1; got {r}, {d}, {depth}")
        qos = QoSProfile(reliability=rel[r], durability=dur[d], history=HistoryPolicy.KEEP_LAST, depth=depth)

        self.peers = {}                  # peer_id -> dict(last_seq, last_rx, up)
        self.seq = 0
        log_dir = self.get_parameter('log_dir').value
        self.rx_csv = self.ev_csv = None
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
            self.rx_csv = csv.writer(open(os.path.join(log_dir, f'mesh_rx_{name}.csv'), 'w', buffering=1))
            self.rx_csv.writerow(['t_rx_s', 'peer_id', 'seq', 'latency_ms', 'gap'])
            self.ev_csv = csv.writer(open(os.path.join(log_dir, f'mesh_events_{name}.csv'), 'w', buffering=1))
            self.ev_csv.writerow(['t_s', 'agent_id', 'peer_id', 'event', 'silence_s'])
        self.pub = self.create_publisher(PeerState, '/mesh/peer_state', qos)
        self.create_subscription(PeerState, '/mesh/peer_state', self.on_peer, qos)
        self.create_timer(1.0 / float(self.get_parameter('heartbeat_hz').value), self.publish_state)
        self.create_timer(0.1, self.check_links)
        self.get_logger().info(f'mesh_node id={self.agent_id} ({name}) peer_state QoS: {qos.reliability.name} {qos.durability.name} '
                               f'{qos.history.name} depth={qos.depth}')

    def publish_state(self):
        sample = self.state_provider()
        if sample is None:
            return                                     # no valid vehicle state yet: say nothing rather than lie
        pos, vel = sample
        m = PeerState()
        m.sender_id, m.seq = self.agent_id, self.seq
        m.stamp_sent = self.get_clock().now().to_msg()
        m.position, m.velocity = pos, vel
        self.seq += 1
        self.pub.publish(m)

    def _event(self, peer, event, silence=0.0):
        t = time.time()
        if self.ev_csv:
            self.ev_csv.writerow([f'{t:.3f}', self.agent_id, peer, event, f'{silence:.3f}'])
        self.get_logger().info(f'link to {peer} {event}' + (f' (silent {silence:.1f} s)' if silence else ''))

    def on_peer(self, m):
        if m.sender_id == self.agent_id:
            return                                     # our own heartbeat, delivered back to our own subscriber
        now = time.time()
        latency_ms = (self.get_clock().now().nanoseconds - (m.stamp_sent.sec * 10**9 + m.stamp_sent.nanosec)) * 1e-6
        p = self.peers.setdefault(m.sender_id, {'last_seq': None, 'last_rx': None, 'up': False})
        gap = 0 if p['last_seq'] is None else max(0, int(m.seq) - p['last_seq'] - 1)
        if p['last_seq'] is None or m.seq > p['last_seq']:
            p['last_seq'] = int(m.seq)
        if not p['up']:
            p['up'] = True
            self._event(m.sender_id, 'UP', 0.0 if p['last_rx'] is None else now - p['last_rx'])
        p['last_rx'] = now
        if self.rx_csv:
            self.rx_csv.writerow([f'{now:.3f}', m.sender_id, m.seq, f'{latency_ms:.3f}', gap])

    def check_links(self):
        now = time.time()
        for peer, p in self.peers.items():
            if p['up'] and now - p['last_rx'] > self.timeout:
                p['up'] = False
                self._event(peer, 'LOST', now - p['last_rx'])


def main(args=None):
    """One process, two DDS contexts: LOCAL (domain 0, localhost-only discovery) for the PX4 state and RADIO (domain 42, bound to radio0 by
    CYCLONEDDS_URI) for the heartbeat; same arrangement and reasoning as swarm_control.swarm_node.main_px4."""
    local_domain = int(os.environ.get('SWARM_LOCAL_DOMAIN', '0'))
    radio_domain = int(os.environ.get('SWARM_RADIO_DOMAIN', '42'))
    prev = os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    ctx_local = Context()
    rclpy.init(args=args, context=ctx_local, domain_id=local_domain)
    if prev is None:
        os.environ.pop('ROS_AUTOMATIC_DISCOVERY_RANGE')
    else:
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = prev
    ctx_radio = Context()
    rclpy.init(args=args, context=ctx_radio, domain_id=radio_domain)

    cli = _cli_params(sys.argv)                       # the local side's parameters are read here, not through a node on the radio domain
    instance = int(cli.get('px4_instance', 0))
    spawn_xy = [float(v) for v in str(cli.get('spawn_xy', '[0.0, 0.0]')).strip('[]').split(',')]
    local = LocalSide(ctx_local, 'leader', instance, spawn_xy)     # role 'leader' = the side that reads PX4's position
    radio = MeshNode(ctx_radio, local.world_state)
    pairs = [(ctx_local, local), (ctx_radio, radio)]
    execs = []
    for ctx, node in pairs:
        ex = SingleThreadedExecutor(context=ctx)
        ex.add_node(node)
        execs.append(ex)
    threads = [threading.Thread(target=ex.spin, daemon=True) for ex in execs]
    for t in threads:
        t.start()
    try:
        while all(t.is_alive() for t in threads):
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    for ex, (ctx, node) in zip(execs, pairs):
        ex.shutdown()
        node.destroy_node()
        ctx.try_shutdown()


if __name__ == '__main__':
    main()
