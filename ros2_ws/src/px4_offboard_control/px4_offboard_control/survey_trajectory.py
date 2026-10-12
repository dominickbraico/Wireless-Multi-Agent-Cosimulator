#!/usr/bin/env python3
"""Fly a lawnmower (boustrophedon) survey over a rectangle, then return and land.

The rectangle has one corner at the arming point and extends `length` metres
North and `width` metres East. Lanes run North-South, spaced `lane_spacing`
apart (pick this from your sensor footprint and desired overlap). Yaw is held
fixed so a downward camera keeps a constant orientation.
"""

import math

from px4_offboard_control.trajectory_base import WaypointTrajectoryNode, run


class SurveyTrajectory(WaypointTrajectoryNode):

    def __init__(self) -> None:
        super().__init__('survey_trajectory')
        self.declare_parameter('length', 30.0)        # lane length, North (m)
        self.declare_parameter('width', 20.0)         # survey width, East (m)
        self.declare_parameter('lane_spacing', 5.0)   # distance between lanes (m)
        self.length = self.get_parameter('length').value
        self.width = self.get_parameter('width').value
        self.lane_spacing = self.get_parameter('lane_spacing').value

    def build_waypoints(self):
        lanes = int(math.floor(self.width / self.lane_spacing)) + 1
        waypoints = []
        for i in range(lanes):
            y = i * self.lane_spacing
            near, far = (0.0, self.length)
            # Alternate direction each lane so we never fly an empty transit leg.
            start, end = (near, far) if i % 2 == 0 else (far, near)
            waypoints += [(start, y), (end, y)]
        waypoints.append((0.0, 0.0))  # return to start before landing
        return waypoints


def main(args=None) -> None:
    run(SurveyTrajectory, args)


if __name__ == '__main__':
    main()
