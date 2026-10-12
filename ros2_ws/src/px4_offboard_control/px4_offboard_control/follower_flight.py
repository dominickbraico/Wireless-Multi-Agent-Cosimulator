#!/usr/bin/env python3
"""Follower flight node: arms, takes off, then tracks the target the swarm node publishes.

The target arrives on the LOCAL DDS domain as /swarm/follower_target (geometry_msgs/PointStamped,
shared world frame, ENU metres). This node never touches the radio. World -> this vehicle's local
NED frame uses the vehicle's spawn pose (its PX4 local origin is where it started):
    north = target.y - spawn_y     east = target.x - spawn_x
Altitude is held at the `altitude` parameter (like every other trajectory node).

Link-loss behaviour is selected with `on_link_lost`:
    hold  (default) keep flying to the last received target
    land  land after `link_timeout_s` without a target
"""
import math

from geometry_msgs.msg import PointStamped
from rclpy.qos import qos_profile_sensor_data

from px4_offboard_control.trajectory_base import TrajectoryNode, run


class FollowerFlight(TrajectoryNode):

    def __init__(self) -> None:
        super().__init__('follower_flight')
        self.declare_parameter('spawn_xy', [0.0, 0.0])      # world ENU (x East, y North) of this vehicle's start
        self.declare_parameter('on_link_lost', 'hold')
        self.declare_parameter('link_timeout_s', 2.0)
        self.spawn = list(self.get_parameter('spawn_xy').value)
        self.on_link_lost = self.get_parameter('on_link_lost').value
        self.link_timeout = float(self.get_parameter('link_timeout_s').value)
        self._target = None            # (north_local, east_local)
        self._t_target = None
        self.create_subscription(PointStamped, '/swarm/follower_target', self._on_target, qos_profile_sensor_data)

    def _on_target(self, m) -> None:
        self._target = (m.point.y - self.spawn[1], m.point.x - self.spawn[0])
        self._t_target = self.get_clock().now()

    def trajectory_step(self):
        if self._target is None:
            return (0.0, 0.0, 0.0)                       # hover at home until the first target arrives
        if self.on_link_lost == 'land' and self._t_target is not None:
            if (self.get_clock().now() - self._t_target).nanoseconds * 1e-9 > self.link_timeout:
                self.get_logger().warn('leader link lost: landing')
                return None
        n, e = self._target
        return (n - self.home_x, e - self.home_y, 0.0)   # base class adds the home offset back


def main(args=None) -> None:
    run(FollowerFlight, args)


if __name__ == '__main__':
    main()
