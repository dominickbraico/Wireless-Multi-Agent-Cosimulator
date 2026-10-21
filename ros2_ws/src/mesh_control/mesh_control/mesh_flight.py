"""Flight node of the mesh-dropout scenario (local DDS domain only; never touches the radio).

role=wander   flies the waypoints of the wanderer's entry in the plan file (orchestrator/mesh_plan.py): seeded random points, one excursion
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
        self.role = self.get_parameter('role').value
        if self.role not in ('wander', 'anchor'):
            raise ValueError(f"role must be wander or anchor, got {self.role}")
        self.spawn = list(self.get_parameter('spawn_xy').value)
        self.land_file = self.get_parameter('land_when_file').value
        self.max_hold = float(self.get_parameter('max_hold_s').value)
        self._plan = None
        self._hold_until = None
        self._t_start = None
        if self.role == 'wander':
            plan = json.load(open(self.get_parameter('plan_file').value))
            self._plan = plan['wanderers'][self.get_parameter('agent_name').value]

    # --- wanderer ---
    def build_waypoints(self):
        return [(y - self.spawn[1], x - self.spawn[0]) for x, y in self._plan['waypoints_enu']]    # (north, east)

    def on_trajectory_start(self) -> None:
        self._t_start = time.time()
        if self.role == 'wander':
            super().on_trajectory_start()
            ex = self._plan['excursion']
            self.get_logger().info(f"plan: excursion drops {ex['k']} link(s) to {ex['dropped_anchors']}; expected flight {self._plan['expected_flight_s']} s")

    def trajectory_step(self):
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
