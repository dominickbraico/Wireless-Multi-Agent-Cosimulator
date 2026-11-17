"""Flight node of the mesh-dropout scenario for a fixed-wing vehicle (local DDS domain only; never touches the radio).

A fixed-wing cannot hover or hold a waypoint, so it does not use offboard position setpoints (px4_offboard_control.trajectory_base). It uses PX4's own
flight modes through vehicle commands:
    take-off      VEHICLE_CMD_NAV_TAKEOFF, then arm (what `commander takeoff` does): PX4's runway take-off along the heading the vehicle was spawned with,
                  to MIS_TAKEOFF_ALT, after which PX4 holds (circles)
    circling      VEHICLE_CMD_DO_REPOSITION to a circle centre (latitude, longitude, altitude above mean sea level); PX4 flies there and circles it with
                  radius NAV_LOITER_RAD (set for the vehicle type in orchestrator/vehicles.py)
The plan (orchestrator/mesh_plan.py, this agent's entry under `agents`) names two circle centres in world ENU metres. Timeline:
    1. take off, fly to the NEAR circle, touch `airborne_file` once on it, wait for `start_file` (the shared start signal of mode roam)
    2. at t_out_s after the start signal: go to the FAR circle (its links drop on the way)
    3. at t_back_s: go back to the NEAR circle (its links recover)
    4. at the plan's t_total_s: log that the plan is complete and keep circling. There is no landing: the run ends with the vehicle in the air.

Frames: the plan is world ENU metres. PX4's local frame is NED with its origin where the estimator started, which is the spawn point, and PX4 reports that
origin's latitude, longitude and altitude (vehicle_local_position ref_lat, ref_lon, ref_alt). A circle centre is converted with a flat-earth approximation around
that origin (error well under 1 m at the few hundred metres used here):
    north = y_world - spawn_y,  east = x_world - spawn_x,  lat = ref_lat + north / R,  lon = ref_lon + east / (R cos(ref_lat)),  alt = ref_alt + altitude
"On a circle" means: horizontal distance to its centre within ON_CIRCLE_TOL_M of the radius, and height within ON_CIRCLE_ALT_TOL_M of the planned altitude.
"""
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition, VehicleStatus

EARTH_RADIUS_M = 6371000.0
ON_CIRCLE_TOL_M = 25.0          # the vehicle overshoots while it joins a circle
ON_CIRCLE_ALT_TOL_M = 10.0
RESEND_S = 2.0                  # commands are repeated until PX4's state shows them accepted (take-off), or three times (reposition)


class MeshPlane(Node):

    def __init__(self) -> None:
        super().__init__('mesh_plane')
        for name, default in (('px4_instance', 0), ('plan_file', ''), ('agent_name', ''), ('spawn_xy', [0.0, 0.0]), ('airborne_file', ''), ('start_file', '')):
            self.declare_parameter(name, default)
        self.instance = int(self.get_parameter('px4_instance').value)
        self.spawn = list(self.get_parameter('spawn_xy').value)
        plan = json.load(open(self.get_parameter('plan_file').value))
        mine = plan['agents'][self.get_parameter('agent_name').value]
        self.radius = float(mine['radius_m'])
        self.altitude = float(mine['altitude_m'])            # metres above the arming point
        self.near, self.far = mine['near_enu'], mine['far_enu']
        self.t_out, self.t_back, self.t_total = float(mine['t_out_s']), float(mine['t_back_s']), float(plan['t_total_s'])
        self._airborne_file = self.get_parameter('airborne_file').value
        self._start_file = self.get_parameter('start_file').value

        ns = '' if self.instance == 0 else f'/px4_{self.instance}'
        self.target_system = self.instance + 1
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)   # must match PX4's uXRCE-DDS bridge
        self._command_pub = self.create_publisher(VehicleCommand, f'{ns}/fmu/in/vehicle_command', qos)
        self.create_subscription(VehicleLocalPosition, f'{ns}/fmu/out/vehicle_local_position_v1', self._on_position, qos)
        self.create_subscription(VehicleStatus, f'{ns}/fmu/out/vehicle_status_v4', self._on_status, qos)
        self.position, self.status = None, None
        self._state = 'WAIT'
        self._last_cmd = 0.0
        self._sends = 0
        self._t0 = None
        self._target = None
        self.create_timer(0.2, self._tick)

    def _on_position(self, msg) -> None:
        self.position = msg

    def _on_status(self, msg) -> None:
        self.status = msg

    def _command(self, command, **params) -> None:
        msg = VehicleCommand()
        msg.command = command
        for k in ('param1', 'param2', 'param3', 'param4', 'param7'):
            setattr(msg, k, float(params.get(k, math.nan)))
        msg.param5 = float(params.get('param5', math.nan))
        msg.param6 = float(params.get('param6', math.nan))
        msg.target_system = self.target_system
        msg.target_component = 1
        msg.source_system = self.target_system
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._command_pub.publish(msg)

    def _offsets(self, centre):
        """(north, east) of a world ENU point relative to this vehicle's local origin (its spawn point)."""
        return centre[1] - self.spawn[1], centre[0] - self.spawn[0]

    def _go_to(self, centre) -> None:
        north, east = self._offsets(centre)
        p = self.position
        lat = p.ref_lat + math.degrees(north / EARTH_RADIUS_M)
        lon = p.ref_lon + math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(p.ref_lat))))
        # param1 ground speed (-1: default), param2 bitmask (1: change mode to hold), param3 circle radius (ignored by PX4 versions that take NAV_LOITER_RAD only)
        self._command(VehicleCommand.VEHICLE_CMD_DO_REPOSITION, param1=-1.0, param2=1.0, param3=self.radius, param5=lat, param6=lon, param7=p.ref_alt + self.altitude)

    def _on_circle(self, centre) -> bool:
        north, east = self._offsets(centre)
        d = math.hypot(self.position.x - north, self.position.y - east)
        return abs(d - self.radius) <= ON_CIRCLE_TOL_M and abs(-self.position.z - self.altitude) <= ON_CIRCLE_ALT_TOL_M

    def _start_leg(self, centre, state, text) -> None:
        self._target, self._state, self._sends, self._last_cmd = centre, state, 0, 0.0
        self.get_logger().info(text)

    def _tick(self) -> None:
        if self.position is None or self.status is None or not self.position.xy_global:
            return
        now = time.time()
        armed = self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        if self._state == 'WAIT':
            if now - self._last_cmd >= RESEND_S:
                self._last_cmd = now
                self._command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF)
                self._command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            if armed and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                self._state = 'TAKEOFF'
                self.get_logger().info('Armed in take-off mode, rolling')
            return
        if not armed:
            if self._state != 'DOWN':
                self._state = 'DOWN'
                self.get_logger().error('Disarmed in flight: the vehicle is down')
            return
        if self._state == 'TAKEOFF':
            if self.status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:      # PX4 holds once the take-off altitude is reached
                self._start_leg(self.near, 'TO_NEAR', f'Take-off complete at {-self.position.z:.0f} m: flying to the near circle')
            return
        if self._sends < 3 and now - self._last_cmd >= RESEND_S and self._target is not None:
            self._last_cmd = now
            self._sends += 1
            self._go_to(self._target)
        if self._state == 'TO_NEAR' and self._on_circle(self.near):
            self._state = 'HOLD_NEAR'
            if self._airborne_file:
                open(self._airborne_file, 'w').close()
            self.get_logger().info(f'On the near circle (radius {self.radius:g} m, {self.altitude:g} m up); waiting for the shared start signal')
        if self._t0 is None:
            if self._state == 'HOLD_NEAR' and self._start_file and os.path.exists(self._start_file):
                self._t0 = now
                self.get_logger().info('start signal: circling until the planned excursion')
            return
        t = now - self._t0
        if self._state == 'HOLD_NEAR' and t >= self.t_out:
            self._start_leg(self.far, 'TO_FAR', 'Excursion: flying to the far circle')
        elif self._state == 'TO_FAR' and self._on_circle(self.far):
            self._state = 'HOLD_FAR'
            self.get_logger().info('On the far circle')
        elif self._state in ('TO_FAR', 'HOLD_FAR') and t >= self.t_back:
            self._start_leg(self.near, 'RETURN', 'Excursion over: flying back to the near circle')
        elif self._state == 'RETURN' and self._on_circle(self.near):
            self._state = 'BACK'
            self.get_logger().info('Back on the near circle')
        elif self._state in ('RETURN', 'BACK') and t >= self.t_total:
            self.get_logger().info(f"Plan complete: {'on' if self._state == 'BACK' else 'NOT yet on'} the near circle; the vehicle keeps circling (no landing)")
            self._state = 'DONE'


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MeshPlane()
    try:
        rclpy.spin(node)
    except (SystemExit, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
