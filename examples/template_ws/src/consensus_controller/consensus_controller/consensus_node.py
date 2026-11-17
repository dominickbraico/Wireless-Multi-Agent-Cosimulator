"""Template controller: altitude consensus over the emulated radio.

Each agent holds one number, its altitude setpoint x_i (metres above its arming point), starting at `start_altitude`.
At `rate_hz` it broadcasts x_i on /swarm/state and moves it toward the mean of the values it has heard recently:

    x_i  <-  x_i + gain * (mean of x_j heard in the last STALE_S seconds - x_i)

This is discrete-time average consensus (R. Olfati-Saber, J. A. Fax, R. M. Murray, "Consensus and Cooperation in
Networked Multi-Agent Systems", Proc. IEEE 95(1), 2007). With every agent in range the values converge to a common
altitude; an agent that hears nobody keeps its own. The vehicle flies to x_i above its take-off point and does not move
horizontally, so agents cannot collide.

What to copy from this file:
  - The node is an ordinary single-domain ROS 2 node. /swarm/state is listed under `scenario.custom.radio_topics` in the
    experiment YAML, so the cosimulator's relay carries it to the other agents; the node does nothing special for that.
  - PX4 is reached on the same domain through `TrajectoryNode` (arming, take-off, landing; ros2_ws/src/px4_offboard_control).
  - Frames: altitude is positive up, relative to the arming point; PX4's local position is NED (z negative up).

Parameters: agent_id, px4_instance, start_altitude (m), gain, run_s (s of consensus before landing), rate_hz, log_dir,
dry_run (no PX4: only the consensus exchange, used by tests/custom_relay.sh).
Output: <log_dir>/consensus.csv  t_s (wall clock), setpoint_m, measured_m (PX4, empty in a dry run), neighbours.
"""
import os
import sys
import time

import rclpy
from consensus_msgs.msg import AgentState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

STALE_S = 1.0     # a neighbour's value older than this is not used
# Must equal the entry in `scenario.custom.radio_topics`: a best-effort publisher does not match a reliable subscriber.
# Depth 10: the topic has no key and several publishers, so depth 1 would let simultaneous messages overwrite each other.
STATE_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)


class Consensus:
    """The consensus exchange, independent of the vehicle. `node.consensus.x` is the current altitude setpoint."""

    def __init__(self, node, measured=lambda: None):
        self.node, self.measured = node, measured
        for name, default in (('agent_id', 1), ('start_altitude', 5.0), ('gain', 0.3), ('run_s', 40.0), ('rate_hz', 5.0), ('log_dir', '')):
            node.declare_parameter(name, default)
        self.agent_id = int(node.get_parameter('agent_id').value)
        self.x = float(node.get_parameter('start_altitude').value)
        self.gain = float(node.get_parameter('gain').value)
        self.heard = {}                 # sender_id -> (altitude_m, time heard)
        self.active = False             # the update runs only while the vehicle is in its consensus phase
        self.pub = node.create_publisher(AgentState, '/swarm/state', STATE_QOS)
        node.create_subscription(AgentState, '/swarm/state', self.on_state, STATE_QOS)
        node.create_timer(1.0 / float(node.get_parameter('rate_hz').value), self.step)
        self.csv = None
        log_dir = node.get_parameter('log_dir').value
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
            self.csv = open(os.path.join(log_dir, 'consensus.csv'), 'w', buffering=1)
            self.csv.write('t_s,setpoint_m,measured_m,neighbours\n')

    def on_state(self, msg):
        if msg.sender_id != self.agent_id:          # the node also receives its own publications
            self.heard[msg.sender_id] = (msg.altitude_m, time.time())

    def step(self):
        if not self.active:
            return
        now = time.time()
        fresh = [alt for alt, t in self.heard.values() if now - t <= STALE_S]
        if fresh:
            self.x += self.gain * (sum(fresh) / len(fresh) - self.x)
        self.pub.publish(AgentState(sender_id=self.agent_id, altitude_m=self.x))
        if self.csv:
            m = self.measured()
            self.csv.write(f'{now:.3f},{self.x:.3f},{"" if m is None else f"{m:.3f}"},{len(fresh)}\n')


def make_flight_node():
    from px4_offboard_control.trajectory_base import TrajectoryNode     # imported here so a dry run needs no PX4 packages

    class ConsensusFlight(TrajectoryNode):
        """Take off to start_altitude, run the consensus for run_s, land."""

        def __init__(self):
            super().__init__('consensus_node')
            self.consensus = Consensus(self, measured=lambda: -float(self.position.z))   # NED z -> metres up
            self.altitude = self.consensus.x          # the base class climbs to self.altitude and holds it
            self.run_s = float(self.get_parameter('run_s').value)
            self.t_start = None

        def on_trajectory_start(self):
            self.t_start = time.time()
            self.consensus.active = True

        def trajectory_step(self):
            if time.time() - self.t_start >= self.run_s:
                self.consensus.active = False
                return None                           # the base class lands and exits
            self.altitude = self.consensus.x          # fly to the consensus value ...
            return 0.0, 0.0, 0.0                      # ... above the take-off point (x, y offsets and yaw)

    return ConsensusFlight()


class ConsensusDryRun(Node):
    """No vehicle: the consensus runs for run_s and the node exits."""

    def __init__(self):
        super().__init__('consensus_node')
        self.consensus = Consensus(self)
        self.consensus.active = True
        self.create_timer(float(self.get_parameter('run_s').value), self.finish)

    def finish(self):
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    probe = Node('consensus_probe')                   # only to read dry_run before choosing the node class
    dry = bool(probe.declare_parameter('dry_run', False).value)
    probe.destroy_node()
    node = ConsensusDryRun() if dry else make_flight_node()
    try:
        rclpy.spin(node)
    except (SystemExit, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
