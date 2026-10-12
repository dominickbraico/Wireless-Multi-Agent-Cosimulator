"""Leader-follower swarm node.

  role=leader   publishes LeaderState on /swarm/leader_state at rate_hz.
  role=follower subscribes, computes target = leader position + offset, publishes
                it locally on /swarm/follower_target, and logs per-message
                latency and sequence gaps to a CSV.

This is the ONLY node that talks across the emulated radio. It never touches PX4;
a local flight node consumes /swarm/follower_target.

Leader position source: `source:=circle` is a synthetic trajectory for link tests
without a simulator. The flight scenario replaces it with the vehicle's real state.
"""
import csv
import math
import os
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from swarm_msgs.msg import LeaderState


class SwarmNode(Node):
    """Radio side of the swarm logic. In `standalone` mode (link tests, no PX4) it is the whole node.

    In `px4` mode it lives in the RADIO DDS context only and is wired to the local side through two hooks:
      state_provider() -> (pos, vel) in the shared world frame, or None if the vehicle has no valid state yet
      target_cb(x, y, z)  hands the follower's target to the local side
    so that nothing about the vehicle is ever published on, or discovered over, the radio domain.
    """

    def __init__(self, context=None, state_provider=None, target_cb=None):
        super().__init__('swarm_node', context=context)
        self.state_provider = state_provider
        self.target_cb = target_cb
        self.declare_parameter('role', 'follower')
        self.declare_parameter('agent_id', 1)
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('offset', [-3.0, 0.0, 0.0])      # follower: target = leader + offset
        self.declare_parameter('source', 'circle')               # leader position source
        self.declare_parameter('circle_center', [0.0, 0.0, 5.0])
        self.declare_parameter('circle_radius', 10.0)
        self.declare_parameter('circle_period_s', 40.0)
        self.declare_parameter('link_timeout_s', 1.0)
        self.declare_parameter('log_csv', '')

        self.role = self.get_parameter('role').value
        self.agent_id = int(self.get_parameter('agent_id').value)
        rate = float(self.get_parameter('rate_hz').value)
        # Best-effort, depth-1 (sensor-style): stale state is useless, retransmitting
        # it only adds load to a lossy shared medium.
        if self.role == 'leader':
            self.pub = self.create_publisher(LeaderState, '/swarm/leader_state', qos_profile_sensor_data)
            self.seq = 0
            self.t0 = self.get_clock().now()
            self.create_timer(1.0 / rate, self.publish_state)
        elif self.role == 'follower':
            self.offset = list(self.get_parameter('offset').value)
            self.timeout = float(self.get_parameter('link_timeout_s').value)
            self.sub = self.create_subscription(
                LeaderState, '/swarm/leader_state', self.on_state, qos_profile_sensor_data)
            self.target_pub = None
            if self.target_cb is None:        # standalone: publish the target on this node's own domain
                self.target_pub = self.create_publisher(PointStamped, '/swarm/follower_target', 10)
            self.last_seq = None
            self.last_rx = None
            self.lost = 0
            self.link_up = False
            self.create_timer(0.2, self.check_link)
            path = self.get_parameter('log_csv').value
            self.csv = None
            if path:
                os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
                self.csv = csv.writer(open(path, 'w', buffering=1))
                self.csv.writerow(['t_rx_s', 'seq', 'latency_ms', 'gap'])
        else:
            raise ValueError(f"role must be leader or follower, got {self.role}")
        self.get_logger().info(f'swarm_node role={self.role} id={self.agent_id}')

    # --- leader ---
    def leader_position(self, t):
        c = self.get_parameter('circle_center').value
        r = float(self.get_parameter('circle_radius').value)
        w = 2 * math.pi / float(self.get_parameter('circle_period_s').value)
        pos = [c[0] + r * math.cos(w * t), c[1] + r * math.sin(w * t), c[2]]
        vel = [-r * w * math.sin(w * t), r * w * math.cos(w * t), 0.0]
        return pos, vel

    def publish_state(self):
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9
        if self.state_provider is not None:
            sample = self.state_provider()
            if sample is None:
                return                        # no valid vehicle state yet: say nothing rather than lie
            pos, vel = sample
        else:
            pos, vel = self.leader_position(t)
        m = LeaderState()
        m.sender_id = self.agent_id
        m.seq = self.seq
        m.stamp_sent = now.to_msg()
        m.position = pos
        m.velocity = vel
        self.seq += 1
        self.pub.publish(m)

    # --- follower ---
    def on_state(self, m):
        now = self.get_clock().now()
        latency_ms = (now.nanoseconds - (m.stamp_sent.sec * 10**9 + m.stamp_sent.nanosec)) * 1e-6
        gap = 0 if self.last_seq is None else int(m.seq) - self.last_seq - 1
        if gap > 0:
            self.lost += gap
        if m.seq > (self.last_seq if self.last_seq is not None else -1):
            self.last_seq = int(m.seq)
            tx = m.position[0] + self.offset[0]
            ty = m.position[1] + self.offset[1]
            tz = m.position[2] + self.offset[2]
            if self.target_cb is not None:
                self.target_cb(tx, ty, tz)
            else:
                tgt = PointStamped()
                tgt.header.stamp = now.to_msg()
                tgt.header.frame_id = 'world'
                tgt.point.x, tgt.point.y, tgt.point.z = tx, ty, tz
                self.target_pub.publish(tgt)
        self.last_rx = now
        if not self.link_up:
            self.link_up = True
            self.get_logger().info('leader link UP')
        if self.csv:
            self.csv.writerow([f'{now.nanoseconds * 1e-9:.6f}', m.seq, f'{latency_ms:.3f}', gap])

    def check_link(self):
        if self.link_up and (self.get_clock().now() - self.last_rx).nanoseconds * 1e-9 > self.timeout:
            self.link_up = False
            self.get_logger().warn(f'leader link LOST (no state for {self.timeout}s)')


class LocalSide(Node):
    """Vehicle side (LOCAL domain, loopback only): reads PX4 state, publishes the follower target.
    PX4 local frame is NED with its origin at the vehicle's start; the shared frame is world ENU, so
        world_x (East) = spawn_x + local_y     world_y (North) = spawn_y + local_x     world_z (Up) = -local_z
    (ground assumed at world z = 0)."""

    def __init__(self, context, role, instance, spawn_xy):
        super().__init__('swarm_local', context=context)
        from px4_msgs.msg import VehicleLocalPosition
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        self.spawn = spawn_xy
        self._lock = threading.Lock()
        self._state = None
        ns = '' if instance == 0 else f'/px4_{instance}'
        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        if role == 'leader':
            self.create_subscription(VehicleLocalPosition, f'{ns}/fmu/out/vehicle_local_position_v1',
                                     self._on_pos, px4_qos)
        else:
            self._pub = self.create_publisher(PointStamped, '/swarm/follower_target', qos_profile_sensor_data)

    def _on_pos(self, m):
        if not (m.xy_valid and m.z_valid):
            return
        with self._lock:
            self._state = ([self.spawn[0] + m.y, self.spawn[1] + m.x, -m.z], [m.vy, m.vx, -m.vz])

    def world_state(self):
        with self._lock:
            return self._state

    def send_target(self, x, y, z):
        t = PointStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.point.x, t.point.y, t.point.z = x, y, z
        self._pub.publish(t)


def _cli_params(argv):
    """Minimal parser for `-p name:=value` pairs after --ros-args."""
    out, i = {}, 0
    while i < len(argv):
        if argv[i] in ('-p', '--param') and i + 1 < len(argv) and ':=' in argv[i + 1]:
            k, v = argv[i + 1].split(':=', 1)
            out[k] = v
            i += 1
        i += 1
    return out


def main_px4(args=None):
    """One process, two DDS contexts: LOCAL (domain 0, localhost-only discovery) and RADIO (domain 42,
    bound to radio0 by CYCLONEDDS_URI). Env is read when each context initialises, so the discovery range is
    switched between the two inits."""
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

    # Parameters the LOCAL side needs are read from the command line directly: creating a node in the
    # radio context just to read them would announce it over radio0.
    cli = _cli_params(sys.argv)
    role = cli.get('role', 'follower')
    instance = int(cli.get('px4_instance', 0))
    spawn_xy = [float(v) for v in str(cli.get('spawn_xy', '[0.0, 0.0]')).strip('[]').split(',')]

    local = LocalSide(ctx_local, role, instance, spawn_xy)
    if role == 'leader':
        radio = SwarmNode(context=ctx_radio, state_provider=local.world_state)
    else:
        radio = SwarmNode(context=ctx_radio, target_cb=local.send_target)
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


def main():
    if os.environ.get('SWARM_MODE', 'standalone') == 'px4':
        return main_px4()
    rclpy.init()
    node = SwarmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
