#!/usr/bin/env python3
"""Print the overview-camera pose that frames every agent of an experiment and the leader's square, as
`x y z qx qy qz qw` for gz's /gui/move_to/pose. Same framing as demo/run_gui_demo.sh.

    overview_camera.py <experiment.yaml> <square side m>
"""
import math
import sys

import yaml

cfg, side = yaml.safe_load(open(sys.argv[1])), float(sys.argv[2])
xs = [a["spawn_pose"][0] for a in cfg["agents"]]
ys = [a["spawn_pose"][1] for a in cfg["agents"]]
x0, x1, y0, y1 = min(xs), max(xs) + side, min(ys), max(ys) + side     # the square extends `side` from the leader's spawn
cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
ex, ey = x1 - x0, y1 - y0
yaw = 0.0 if ey >= ex else math.pi / 2          # yaw 0: screen-horizontal is world y; yaw 90 deg: world x
long_, short = (ey, ex) if ey >= ex else (ex, ey)
h = max(1.15 * max(long_ / 2 / math.tan(math.radians(30)), short / 2 / 0.325), 20.0)   # 60 deg horizontal, 16:9 window
pitch = math.pi / 2                             # straight down
cy_, sy_, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
print(cx, cy, h, -sy_ * sp, cy_ * sp, sy_ * cp, cy_ * cp)
