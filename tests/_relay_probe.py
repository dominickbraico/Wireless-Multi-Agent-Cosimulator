#!/usr/bin/env python3
"""Publisher and subscriber used by tests/custom_relay.sh (std_msgs/String on the domain given by ROS_DOMAIN_ID).

    _relay_probe.py pub <topic> <count> <prefix> <wait_subs> [reliability durability depth]
        waits (at most 15 s) until <wait_subs> subscriptions are matched, then 1 s more, then publishes "<prefix>-<i>" at 20 Hz
    _relay_probe.py sub <topic> <seconds> [reliability durability depth]
        prints every received string, one per line, for <seconds>
"""
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def qos(args):
    rel, dur, depth = (args + ["reliable", "volatile", "200"])[:3]
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE if rel == "reliable" else ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL if dur == "transient_local" else DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST, depth=int(depth))


def main():
    mode, topic = sys.argv[1], sys.argv[2]
    rclpy.init()
    node = rclpy.create_node(f"relay_probe_{mode}")
    if mode == "pub":
        count, prefix, wait_subs = int(sys.argv[3]), sys.argv[4], int(sys.argv[5])
        pub = node.create_publisher(String, topic, qos(sys.argv[6:]))
        t0 = time.time()
        while pub.get_subscription_count() < wait_subs and time.time() - t0 < 15:
            rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(1.0)
        for i in range(count):
            pub.publish(String(data=f"{prefix}-{i}"))
            time.sleep(0.05)
        time.sleep(1.0)                      # let a reliable writer finish delivering before the process exits
    else:
        seconds = float(sys.argv[3])
        node.create_subscription(String, topic, lambda m: print(m.data, flush=True), qos(sys.argv[4:]))
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
