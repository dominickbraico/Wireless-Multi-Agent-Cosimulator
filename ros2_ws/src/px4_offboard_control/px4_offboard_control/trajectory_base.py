"""Shared scaffolding for trajectory nodes.

TrajectoryNode owns everything that is the same for every mission: QoS, the
PX4 topics, the offboard heartbeat, arming, takeoff, and landing. A subclass
only has to say *where the vehicle should be* by implementing
`trajectory_step()`.

Mission phases (self._state):
    WARMUP      stream setpoints, then request offboard + arm until PX4 agrees
    TAKEOFF     climb straight up to `altitude`
    TRAJECTORY  call trajectory_step() every tick until it returns None
    LANDING     command land, exit once PX4 reports disarmed

All trajectory coordinates are offsets (metres, NED: x=North, y=East) from the
position the vehicle armed at. Altitude is a positive "metres up" parameter and
converted to NED z internally.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

WARMUP, TAKEOFF, TRAJECTORY, LANDING = 'WARMUP', 'TAKEOFF', 'TRAJECTORY', 'LANDING'


class TrajectoryNode(Node):
    RATE_HZ = 20.0
    WARMUP_TICKS = 10           # setpoints PX4 must see before accepting offboard
    RETRY_TICKS = 20            # re-send offboard/arm request once per second
    TAKEOFF_TOLERANCE = 0.3     # metres

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.declare_parameter('altitude', 5.0)   # metres above start, positive up
        self.declare_parameter('speed', 2.0)      # m/s along the path
        # Multi-vehicle: PX4 instance N publishes under /px4_N for N > 0 and uses MAV_SYS_ID N+1
        # (verified on the pinned PX4 commit; instance 0 has no namespace and sys id 1).
        self.declare_parameter('px4_instance', 0)
        self.instance = int(self.get_parameter('px4_instance').value)
        ns = '' if self.instance == 0 else f'/px4_{self.instance}'
        self.target_system = self.instance + 1
        self.altitude = self.get_parameter('altitude').value
        self.speed = self.get_parameter('speed').value
        self.dt = 1.0 / self.RATE_HZ

        # Must match PX4's uXRCE-DDS bridge or no messages flow.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._mode_pub = self.create_publisher(
            OffboardControlMode, f'{ns}/fmu/in/offboard_control_mode', qos)
        self._setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f'{ns}/fmu/in/trajectory_setpoint', qos)
        self._command_pub = self.create_publisher(
            VehicleCommand, f'{ns}/fmu/in/vehicle_command', qos)
        self.create_subscription(
            VehicleLocalPosition, f'{ns}/fmu/out/vehicle_local_position_v1',
            self._on_position, qos)
        self.create_subscription(
            VehicleStatus, f'{ns}/fmu/out/vehicle_status_v4', self._on_status, qos)

        self.position = VehicleLocalPosition()
        self.status = VehicleStatus()
        self._have_position = False
        self._have_status = False

        self.home_x = 0.0
        self.home_y = 0.0
        self._state = WARMUP
        self._tick_count = 0
        self._land_sent = False

        self.timer = self.create_timer(self.dt, self._tick)

    # ------------------------------------------------------------------
    # Hooks for subclasses
    # ------------------------------------------------------------------
    def on_trajectory_start(self) -> None:
        """Called once, when takeoff completes. Reset any trajectory state here."""

    def trajectory_step(self):
        """Return (x_offset, y_offset, yaw) for this tick, or None when finished."""
        raise NotImplementedError

    # ------------------------------------------------------------------
    # Helpers available to subclasses
    # ------------------------------------------------------------------
    def distance_to(self, x_offset: float, y_offset: float) -> float:
        """Horizontal distance from the vehicle to an offset-from-home point."""
        return math.hypot(self.position.x - self.home_x - x_offset,
                          self.position.y - self.home_y - y_offset)

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------
    def _on_position(self, msg) -> None:
        self.position = msg
        self._have_position = True

    def _on_status(self, msg) -> None:
        self.status = msg
        self._have_status = True

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------
    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _publish_heartbeat(self) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._now_us()
        self._mode_pub.publish(msg)

    def _publish_setpoint(self, x_offset: float, y_offset: float, yaw: float) -> None:
        msg = TrajectorySetpoint()
        msg.position = [self.home_x + x_offset, self.home_y + y_offset, -self.altitude]
        msg.velocity = [math.nan] * 3
        msg.acceleration = [math.nan] * 3
        msg.yaw = float(yaw)
        msg.timestamp = self._now_us()
        self._setpoint_pub.publish(msg)

    def _send_command(self, command, param1=0.0, param2=0.0) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = self.target_system
        msg.target_component = 1
        msg.source_system = self.target_system
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._now_us()
        self._command_pub.publish(msg)

    # ------------------------------------------------------------------
    # Mission state machine
    # ------------------------------------------------------------------
    def _tick(self) -> None:
        self._publish_heartbeat()
        self._tick_count += 1

        if self._state == WARMUP:
            self._publish_setpoint(0.0, 0.0, 0.0)
            if not (self._have_position and self._have_status):
                return  # nothing valid from PX4 yet
            if self._tick_count < self.WARMUP_TICKS:
                return
            in_offboard = self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            armed = self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            if in_offboard and armed:
                self.home_x, self.home_y = self.position.x, self.position.y
                self.get_logger().info('Armed in offboard mode, taking off')
                self._state = TAKEOFF
            elif self._tick_count % self.RETRY_TICKS == 0:
                self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        elif self._state == TAKEOFF:
            self._publish_setpoint(0.0, 0.0, 0.0)
            if abs(self.position.z + self.altitude) < self.TAKEOFF_TOLERANCE:
                self.get_logger().info('Reached altitude, starting trajectory')
                self.on_trajectory_start()
                self._state = TRAJECTORY

        elif self._state == TRAJECTORY:
            step = self.trajectory_step()
            if step is None:
                self.get_logger().info('Trajectory complete, landing')
                self._state = LANDING
            else:
                self._publish_setpoint(*step)

        elif self._state == LANDING:
            if not self._land_sent:
                self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self._land_sent = True
            if self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.get_logger().info('Landed and disarmed')
                raise SystemExit


class WaypointTrajectoryNode(TrajectoryNode):
    """Flies a list of (x, y) offsets at constant speed, then lands.

    The setpoint ("carrot") moves toward the current waypoint at `speed` m/s
    instead of jumping to it, so speed is controlled by us rather than by
    PX4's velocity limits. A waypoint counts as reached once the carrot has
    arrived *and* the vehicle is within `acceptance_radius`.
    """

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.declare_parameter('acceptance_radius', 0.5)
        self.acceptance_radius = self.get_parameter('acceptance_radius').value
        self._waypoints = []
        self._index = 0
        self._carrot = (0.0, 0.0)

    def build_waypoints(self):
        """Return a list of (x_offset, y_offset) tuples. Implemented by subclasses."""
        raise NotImplementedError

    def on_trajectory_start(self) -> None:
        self._waypoints = self.build_waypoints()
        self._index = 0
        self._carrot = (0.0, 0.0)
        self.get_logger().info(f'Flying {len(self._waypoints)} waypoints at {self.speed} m/s')

    def trajectory_step(self):
        if self._index >= len(self._waypoints):
            return None
        tx, ty = self._waypoints[self._index]
        cx, cy = self._carrot
        dx, dy = tx - cx, ty - cy
        dist = math.hypot(dx, dy)
        max_step = self.speed * self.dt
        if dist <= max_step:
            self._carrot = (tx, ty)
            if self.distance_to(tx, ty) < self.acceptance_radius:
                self._index += 1
        else:
            self._carrot = (cx + dx / dist * max_step, cy + dy / dist * max_step)
        return (*self._carrot, 0.0)


def run(node_class, args=None) -> None:
    """Shared main(): init, spin, clean shutdown."""
    rclpy.init(args=args)
    node = node_class()
    try:
        rclpy.spin(node)
    except (SystemExit, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
