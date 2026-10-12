#!/usr/bin/env python3
"""Fly a circle, then land.

Unlike the waypoint nodes this trajectory is parametric: position is a
function of elapsed time. The circle passes through the arming point (its
centre is `radius` metres East of it) so the first setpoint is continuous with
where the vehicle already is. Flown clockwise seen from above, nose along the
direction of travel.

Parameters: radius (m), revolutions, plus altitude/speed. `speed` is the
tangential speed, so angular rate = speed / radius.
"""

import math

from px4_offboard_control.trajectory_base import TrajectoryNode, run


class CircleTrajectory(TrajectoryNode):

    def __init__(self) -> None:
        super().__init__('circle_trajectory')
        self.declare_parameter('radius', 5.0)
        self.declare_parameter('revolutions', 2.0)
        self.radius = self.get_parameter('radius').value
        self.revolutions = self.get_parameter('revolutions').value
        self._elapsed = 0.0

    def on_trajectory_start(self) -> None:
        self._elapsed = 0.0
        omega = self.speed / self.radius
        self._duration = self.revolutions * 2.0 * math.pi / omega
        self.get_logger().info(
            f'Circle r={self.radius} m, {self.revolutions} rev, {self._duration:.1f} s')

    def trajectory_step(self):
        if self._elapsed > self._duration:
            return None
        omega = self.speed / self.radius
        theta = -math.pi / 2.0 + omega * self._elapsed   # start at the bottom of the circle
        x = self.radius * math.cos(theta)
        y = self.radius + self.radius * math.sin(theta)
        yaw = math.atan2(math.cos(theta), -math.sin(theta))  # tangent, North=0, East=+
        self._elapsed += self.dt
        return (x, y, yaw)


def main(args=None) -> None:
    run(CircleTrajectory, args)


if __name__ == '__main__':
    main()
