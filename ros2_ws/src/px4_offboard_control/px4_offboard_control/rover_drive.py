#!/usr/bin/env python3
"""PX4 rover control: RoverNode (arming, offboard heartbeat, setpoints, disarming) and RoverDrive, which drives a square and disarms.

RoverNode is the ground-vehicle counterpart of trajectory_base.TrajectoryNode: it arms in offboard mode, calls `drive_step()` every tick for the
next target, and disarms when that returns None. RoverDrive (this file, `rover_drive`) visits the corners North, East, South, West of the arming
point (offsets in metres, NED: x = North, y = East) and ends at the start. Parameters: side (m), speed (m/s), acceptance_radius (m),
px4_instance, log_csv. mesh_control.mesh_drive is the other subclass.

Control: the generic `TrajectorySetpoint` (position, NED x/y) with only the `position` flag set in OffboardControlMode. At the PX4 commit
pinned in px4/Dockerfile the rover module turns that into its own `rover_position_setpoint` (DifferentialOffboardMode.cpp, offboardControl():
position -> start and cruising speed NaN, i.e. the vehicle's position and the maximum speed). It does so continuously, so a
`RoverPositionSetpoint` published by this node over uXRCE-DDS would be overwritten (observed: the rover armed and did not move). PX4's rover documentation lists the
`RoverPositionSetpoint` route, but in this build it is only usable from a registered external mode (PX4 ROS 2 interface library). Re-check this when the PX4 commit changes.
The speed cannot be set through this route, so the setpoint is a "carrot" that moves along each leg at `speed`, as in trajectory_base.WaypointTrajectoryNode;
the rover follows it at no more than its own limit (RO_SPEED_LIM). There is no altitude: rover modules ignore z.
"""
import csv
import math
import os

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_offboard_control.trajectory_base import run          # shared main: init, spin, clean shutdown
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

WARMUP, DRIVE, STOP = 'WARMUP', 'DRIVE', 'STOP'


class RoverNode(Node):
    RATE_HZ = 20.0
    WARMUP_TICKS = 10           # setpoints PX4 must see before it accepts offboard
    RETRY_TICKS = 20            # re-send the mode/arm (or disarm) request once per second
    DONE_MARKER = 'Route complete and disarmed'     # logged when the vehicle has disarmed after its route (the supervisors wait for it)

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.declare_parameter('acceptance_radius', 0.5)     # m; the airframe's own arrival radius (NAV_ACC_RAD) is also 0.5
        self.declare_parameter('px4_instance', 0)            # instance N publishes under /px4_N for N > 0 and uses MAV_SYS_ID N+1
        self.declare_parameter('log_csv', '')
        self.accept = float(self.get_parameter('acceptance_radius').value)
        self.instance = int(self.get_parameter('px4_instance').value)
        ns = '' if self.instance == 0 else f'/px4_{self.instance}'
        self.target_system = self.instance + 1

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)        # must match PX4's uXRCE-DDS bridge
        self._mode_pub = self.create_publisher(OffboardControlMode, f'{ns}/fmu/in/offboard_control_mode', qos)
        self._sp_pub = self.create_publisher(TrajectorySetpoint, f'{ns}/fmu/in/trajectory_setpoint', qos)
        self._cmd_pub = self.create_publisher(VehicleCommand, f'{ns}/fmu/in/vehicle_command', qos)
        self.create_subscription(VehicleLocalPosition, f'{ns}/fmu/out/vehicle_local_position_v1', self._on_position, qos)
        self.create_subscription(VehicleStatus, f'{ns}/fmu/out/vehicle_status_v4', self._on_status, qos)

        self.position = None
        self.status = None
        self.home = (0.0, 0.0)
        self.leg = 0                                                      # logged in the CSV: which leg or waypoint the route is on
        self._state = WARMUP
        self._tick_count = 0

        self.csv = None
        path = self.get_parameter('log_csv').value
        if path:
            os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
            self.csv = csv.writer(open(path, 'w', buffering=1))
            self.csv.writerow(['t_s', 'x_north_m', 'y_east_m', 'leg', 'state'])
        self.t0 = self.get_clock().now()
        self.create_timer(1.0 / self.RATE_HZ, self._tick)

    def _on_position(self, msg) -> None:
        self.position = msg

    def _on_status(self, msg) -> None:
        self.status = msg

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _heartbeat(self) -> None:
        m = OffboardControlMode()
        m.position = True
        m.velocity = m.acceleration = m.attitude = m.body_rate = False
        m.timestamp = self._now_us()
        self._mode_pub.publish(m)

    def _setpoint(self, target) -> None:
        m = TrajectorySetpoint()
        m.position = [float(target[0]), float(target[1]), 0.0]               # NED; z is ignored by the rover modules
        m.velocity = [math.nan] * 3
        m.acceleration = [math.nan] * 3
        m.yaw = math.nan
        m.timestamp = self._now_us()
        self._sp_pub.publish(m)

    def _command(self, command, p1=0.0, p2=0.0) -> None:
        m = VehicleCommand()
        m.command, m.param1, m.param2 = command, p1, p2
        m.target_system, m.target_component = self.target_system, 1
        m.source_system, m.source_component = self.target_system, 1
        m.from_external = True
        m.timestamp = self._now_us()
        self._cmd_pub.publish(m)

    def _tick(self) -> None:
        self._heartbeat()
        self._tick_count += 1
        if self.position is None or self.status is None:
            return                                                        # nothing valid from PX4 yet
        px, py = self.position.x, self.position.y
        if self.csv and self._tick_count % 4 == 0:
            self.csv.writerow([f'{(self.get_clock().now() - self.t0).nanoseconds * 1e-9:.2f}', f'{px:.3f}', f'{py:.3f}', self.leg, self._state])

        armed = self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        if self._state == WARMUP:
            self._setpoint((px, py))                                       # hold where it is
            if self._tick_count < self.WARMUP_TICKS:
                return
            if armed and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.home = (px, py)
                self.get_logger().info('Armed in offboard mode')
                self.on_drive_start(px, py)
                self._state = DRIVE
            elif self._tick_count % self.RETRY_TICKS == 0:
                self._command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)          # custom mode 6 = offboard
                self._command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        elif self._state == DRIVE:
            target = self.drive_step(px, py)
            if target is None:
                self._state = STOP
            else:
                self._setpoint(target)

        elif self._state == STOP:
            self._setpoint(self.home)
            if not armed:
                self.get_logger().info(self.DONE_MARKER)
                raise SystemExit
            if self._tick_count % self.RETRY_TICKS == 0:
                self._command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)

    # Hooks for subclasses ------------------------------------------------
    def on_drive_start(self, px: float, py: float) -> None:
        """Called once, when the vehicle is armed in offboard mode. self.home is its local position then (absolute NED metres)."""

    def drive_step(self, px: float, py: float):
        """Return the absolute NED (north, east) target for this tick, or None when the route is finished. (px, py) is the vehicle's position."""
        raise NotImplementedError


class RoverDrive(RoverNode):

    def __init__(self) -> None:
        super().__init__('rover_drive')
        self.declare_parameter('side', 6.0)
        self.declare_parameter('speed', 1.0)
        self.side = float(self.get_parameter('side').value)
        self.speed = float(self.get_parameter('speed').value)
        s = self.side
        self.corners = [(s, 0.0), (s, s), (0.0, s), (0.0, 0.0)]          # offsets from the arming point, NED metres
        self.carrot = (0.0, 0.0)                                          # the moving setpoint (absolute NED metres)

    def on_drive_start(self, px, py) -> None:
        self.carrot = (px, py)
        self.get_logger().info('Driving the square')

    def drive_step(self, px, py):
        cx, cy = self.corners[self.leg]
        target = (self.home[0] + cx, self.home[1] + cy)
        kx, ky = self.carrot
        d = math.hypot(target[0] - kx, target[1] - ky)
        step = self.speed / self.RATE_HZ
        self.carrot = target if d <= step else (kx + (target[0] - kx) / d * step, ky + (target[1] - ky) / d * step)
        if self.carrot == target and math.hypot(px - target[0], py - target[1]) <= self.accept:
            self.get_logger().info(f'Reached corner {self.leg + 1} of {len(self.corners)}')
            self.leg += 1
            if self.leg >= len(self.corners):
                return None
        return self.carrot


def main(args=None) -> None:
    run(RoverDrive, args)


if __name__ == '__main__':
    main()
