"""Ground-vehicle flight node of the mesh-dropout scenario (a rover; local DDS domain only; never touches the radio).

The rover counterpart of mesh_flight.py role=roam: it follows this agent's seeded random path from the plan file (orchestrator/mesh_plan.py) on the
SHARED timeline. After arming in offboard mode it touches `airborne_file` (the same file name the drones use when they have reached their altitude,
which is what the supervisor waits for) and stays where it is until `start_file` exists; the target is then a function of the time since that moment,
never of where the vehicle is, so the planned timeline holds. At the plan's end time (`t_total_s`) it stops at its spawn point and disarms.

The plan is in world ENU metres; the PX4 local frame is NED with its origin where the vehicle armed (its spawn point), so
    north = y_world - spawn_y        east = x_world - spawn_x
exactly as in mesh_flight. Built on px4_offboard_control.rover_drive.RoverNode (arming, offboard setpoints, disarming).
"""
import json
import os
import time

from px4_offboard_control.rover_drive import RoverNode
from px4_offboard_control.trajectory_base import run


class MeshDrive(RoverNode):

    def __init__(self) -> None:
        super().__init__('mesh_drive')
        self.declare_parameter('plan_file', '')
        self.declare_parameter('agent_name', '')
        self.declare_parameter('spawn_xy', [0.0, 0.0])      # world ENU (x East, y North) of this vehicle's start
        self.declare_parameter('airborne_file', '')         # touched when this vehicle is armed and ready (name kept from the drones)
        self.declare_parameter('start_file', '')            # the drive starts when this file exists
        spawn = list(self.get_parameter('spawn_xy').value)
        plan = json.load(open(self.get_parameter('plan_file').value))
        mine = plan['agents'][self.get_parameter('agent_name').value]
        self._times = list(mine['times_s'])
        self._ned = [(y - spawn[1], x - spawn[0]) for x, y in mine['waypoints_enu']]          # (north, east) offsets from home
        self._t_total = float(plan['t_total_s'])
        self._airborne_file = self.get_parameter('airborne_file').value
        self._start_file = self.get_parameter('start_file').value
        self._t0 = None

    def on_drive_start(self, px, py) -> None:
        if self._airborne_file:
            open(self._airborne_file, 'w').close()
        self.get_logger().info(f'ready; waiting for the shared start signal ({self._start_file}); the drive lasts {self._t_total:.0f} s')

    def drive_step(self, px, py):
        h = self.home
        if self._t0 is None:
            if self._start_file and os.path.exists(self._start_file):
                self._t0 = time.time()
                self.get_logger().info('start signal: driving the planned path')
            return h                                           # wait at the arming point until every agent is ready
        t = time.time() - self._t0
        if t >= self._t_total:
            self.get_logger().info('planned time over: stopping at the spawn point')
            return None
        ts, pts = self._times, self._ned
        if t >= ts[-1]:
            return (h[0] + pts[-1][0], h[1] + pts[-1][1])      # back home, waiting for the shared end time
        i = max(j for j in range(len(ts)) if ts[j] <= t)
        self.leg = i
        f = (t - ts[i]) / (ts[i + 1] - ts[i])
        return (h[0] + pts[i][0] + f * (pts[i + 1][0] - pts[i][0]), h[1] + pts[i][1] + f * (pts[i + 1][1] - pts[i][1]))


def main(args=None) -> None:
    run(MeshDrive, args)


if __name__ == '__main__':
    main()
