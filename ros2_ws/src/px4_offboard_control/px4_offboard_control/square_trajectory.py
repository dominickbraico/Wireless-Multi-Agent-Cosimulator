#!/usr/bin/env python3
"""Fly a square, then land.

The square starts and ends at the arming point and goes North, East, South,
West. Parameters: side (m), laps, plus altitude/speed/acceptance_radius.
"""

from px4_offboard_control.trajectory_base import WaypointTrajectoryNode, run


class SquareTrajectory(WaypointTrajectoryNode):

    def __init__(self) -> None:
        super().__init__('square_trajectory')
        self.declare_parameter('side', 10.0)
        self.declare_parameter('laps', 1)
        self.side = self.get_parameter('side').value
        self.laps = self.get_parameter('laps').value

    def build_waypoints(self):
        s = self.side
        corners = [(s, 0.0), (s, s), (0.0, s), (0.0, 0.0)]
        return corners * self.laps


def main(args=None) -> None:
    run(SquareTrajectory, args)


if __name__ == '__main__':
    main()
