"""Flight node of the mesh-dropout scenario (local DDS domain only; never touches the radio).

role=roam     (mode roam) flies this agent's own seeded random path on a SHARED timeline: after take-off it touches `airborne_file` and hovers until `start_file`
              exists (every agent then starts together); the setpoint is a function of the time since that moment, never of where the vehicle is, so the
              planned timeline holds for every agent; at `t_total_s` it lands at its spawn point.
role=wander   (mode excursion) flies the waypoints of the wanderer's entry in the plan file (orchestrator/mesh_plan.py): seeded random points, one excursion
              point where it hovers `dwell_s`, more random points, then its spawn point, then it lands.
role=anchor   takes off, hovers at its spawn point, and lands when `land_when_file` exists (the supervisor creates it when every wanderer
              has landed) or after `max_hold_s` of hovering, whichever comes first.

The plan is in world ENU metres; this vehicle's PX4 local frame is NED with its origin where it armed (its spawn point), so
    north = y_world - spawn_y        east = x_world - spawn_x
exactly as in px4_offboard_control.follower_flight. Altitude is the `altitude` parameter (metres above the arming point); every agent has its own
altitude in the plan so the paths cannot collide. Built on px4_offboard_control.trajectory_base (arming, take-off, carrot-following, landing).
"""
import json
import os
import time

from px4_offboard_control.trajectory_base import WaypointTrajectoryNode, run


class MeshFlight(WaypointTrajectoryNode):

    def __init__(self) -> None:
        super().__init__('mesh_flight')
        self.declare_parameter('role', 'anchor')
        self.declare_parameter('plan_file', '')
        self.declare_parameter('agent_name', '')
        self.declare_parameter('spawn_xy', [0.0, 0.0])      # world ENU (x East, y North) of this vehicle's start
        self.declare_parameter('land_when_file', '')
        self.declare_parameter('max_hold_s', 600.0)
        self.declare_parameter('airborne_file', '')         # roam: touched when this vehicle has reached its altitude
        self.declare_parameter('start_file', '')            # roam: the flight starts when this file exists
        self.role = self.get_parameter('role').value
        if self.role not in ('wander', 'anchor', 'roam'):
            raise ValueError(f"role must be roam, wander or anchor, got {self.role}")
        self.spawn = list(self.get_parameter('spawn_xy').value)
        self.land_file = self.get_parameter('land_when_file').value
        self.max_hold = float(self.get_parameter('max_hold_s').value)
        self._plan = None
        self._hold_until = None
        self._t_start = None
        self._t0 = None
        if self.role == 'wander':
            plan = json.load(open(self.get_parameter('plan_file').value))
            self._plan = plan['wanderers'][self.get_parameter('agent_name').value]
        elif self.role == 'roam':
            plan = json.load(open(self.get_parameter('plan_file').value))
            mine = plan['agents'][self.get_parameter('agent_name').value]
            self._times = list(mine['times_s'])
            self._ned = [(y - self.spawn[1], x - self.spawn[0]) for x, y in mine['waypoints_enu']]     # (north, east) offsets from home
            self._t_total = float(plan['t_total_s'])
            self._airborne_file = self.get_parameter('airborne_file').value
            self._start_file = self.get_parameter('start_file').value

    # --- wanderer ---
    def build_waypoints(self):
        return [(y - self.spawn[1], x - self.spawn[0]) for x, y in self._plan['waypoints_enu']]    # (north, east)

    def on_trajectory_start(self) -> None:
        self._t_start = time.time()
        if self.role == 'roam':
            if self._airborne_file:
                open(self._airborne_file, 'w').close()
            self.get_logger().info(f'airborne; waiting for the shared start signal ({self._start_file}); flight lasts {self._t_total:.0f} s')
            return
        if self.role == 'wander':
            super().on_trajectory_start()
            ex = self._plan['excursion']
            self.get_logger().info(f"plan: excursion drops {ex['k']} link(s) to {ex['dropped_anchors']}; expected flight {self._plan['expected_flight_s']} s")

    def _roam_step(self):
        if self._t0 is None:
            if self._start_file and os.path.exists(self._start_file):
                self._t0 = time.time()
                self.get_logger().info('start signal: flying the planned path')
            return (0.0, 0.0, 0.0)                        # hover at the arming point until every agent is airborne
        t = time.time() - self._t0
        if t >= self._t_total:
            self.get_logger().info('planned flight time over: landing at the spawn point')
            return None
        ts, pts = self._times, self._ned
        if t >= ts[-1]:
            return (*pts[-1], 0.0)                        # back home, hovering until the shared end time
        i = max(j for j in range(len(ts)) if ts[j] <= t)
        f = (t - ts[i]) / (ts[i + 1] - ts[i])
        return (pts[i][0] + f * (pts[i + 1][0] - pts[i][0]), pts[i][1] + f * (pts[i + 1][1] - pts[i][1]), 0.0)

    def trajectory_step(self):
        if self.role == 'roam':
            return self._roam_step()
        if self.role == 'anchor':
            held = time.time() - self._t_start
            if (self.land_file and os.path.exists(self.land_file)) or held > self.max_hold:
                self.get_logger().info('anchor: landing' + (' (wanderers are down)' if os.path.exists(self.land_file or '/nonexistent') else ' (hold limit)'))
                return None
            return (0.0, 0.0, 0.0)                        # hover at the arming point
        if self._hold_until is not None:                  # dwelling at the excursion point
            if time.time() < self._hold_until:
                return (*self._carrot, 0.0)
            self._hold_until = None
            self.get_logger().info('excursion dwell over: flying back')
        before = self._index
        step = super().trajectory_step()
        if step is not None and self._index > before:     # waypoint `before` was just reached
            label = self._plan['labels'][before]
            self.get_logger().info(f"reached waypoint {before + 1}/{len(self._waypoints)} ({label})")
            dwell = float(self._plan['dwell_s'].get(str(before), 0.0))
            if dwell > 0:
                self._hold_until = time.time() + dwell
                self.get_logger().info(f'holding {dwell:g} s at the {label} point')
        return step


def main(args=None) -> None:
    run(MeshFlight, args)


if __name__ == '__main__':
    main()
