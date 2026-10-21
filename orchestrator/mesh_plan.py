#!/usr/bin/env python3
"""Flight plan of the mesh-dropout scenario: seeded random waypoints plus one excursion that is guaranteed to drop 1 or 2 links.

    mesh_plan.py <experiment.yaml> [--out plan.json]      print (and optionally save) the plan the YAML produces

Purpose: show links dropping and recovering between agents (docs/MESH_DROPOUT.md). One or more agents (`wanderers`) fly; the others
(`anchors`) hover at their spawn points. The plan is a pure function of the YAML (including the seed), so every agent container computes
the same plan and a run can be repeated exactly.

Wanderer path, in order:
  1. `random_waypoints` random points inside `roam_radius_m` of its spawn. Every anchor is farther than `margin_m` inside the dropout range
     there, so no link drops during this phase.
  2. An outbound point just outside the roaming area on the excursion heading, then the EXCURSION target: a point where exactly k anchors are more than `margin_m` beyond the dropout range D and all the other anchors are
     more than `margin_m` inside it (k drawn from `excursion_links`). The wanderer hovers there `dwell_s` seconds. This is what drops k links.
  3. The outbound point again, `random_waypoints` more random points (links recover), then home (its spawn point), then it lands.
The excursion heading is drawn from the seed over the whole circle, from `heading_sectors_deg` arcs when given (use it to keep the excursion clear of terrain or
obstacles in a world), or fixed with `heading_deg`. D is the distance at which the YAML's own signal model reaches the receiver floor (orchestrator/linkmodel.py), taken in 3-D because the agents
fly at different altitudes (each agent i flies at altitude_base_m + i * altitude_step_m above its arming point, so paths cannot collide).

Modelling notes: the margin has to cover the wanderer's tracking error (a vehicle lags its setpoint by roughly speed x 1 s while moving; it is
stationary during the dwell). Ground is assumed flat for the altitude arithmetic, as elsewhere. Waypoints are ENU world metres.
"""
import argparse
import json
import math
import random
import sys

import yaml

import linkmodel

DEFAULTS = {
    "seed": 1,
    "wanderers": None,            # default: the last agent in the YAML
    "altitude_base_m": 8.0,
    "altitude_step_m": 3.0,
    "speed_mps": 3.0,
    "random_waypoints": 3,
    "roam_radius_m": 20.0,
    "excursion_links": [1, 2],
    "margin_m": 3.0,
    "dwell_s": 10.0,
    "heading_deg": None,          # fixed excursion heading (ENU: 0 = east, 90 = north); default: drawn from the seed
    "heading_sectors_deg": None,  # or: draw the heading from these [from, to] arcs only (degrees, ENU), e.g. to keep clear of terrain
    "max_excursion_m": 250.0,
    "heartbeat_hz": 5.0,
    "link_timeout_s": 1.5,
}


def settings(cfg):
    s = dict(DEFAULTS)
    given = ((cfg.get("scenario") or {}).get("mesh_dropout")) or {}
    unknown = set(given) - set(DEFAULTS)
    if unknown:
        raise ValueError(f"scenario.mesh_dropout: unknown keys {sorted(unknown)}")
    s.update(given)
    names = [a["name"] for a in cfg["agents"]]
    if s["wanderers"] is None:
        s["wanderers"] = [names[-1]]
    bad = [w for w in s["wanderers"] if w not in names]
    if bad:
        raise ValueError(f"scenario.mesh_dropout.wanderers: unknown agents {bad}")
    if len(s["wanderers"]) >= len(names):
        raise ValueError("at least one agent must stay behind as an anchor")
    if not s["excursion_links"] or any(int(k) < 1 or int(k) > len(names) - len(s["wanderers"]) for k in s["excursion_links"]):
        raise ValueError(f"excursion_links {s['excursion_links']} must be between 1 and the number of anchors ({len(names) - len(s['wanderers'])})")
    for k in ("speed_mps", "roam_radius_m", "dwell_s", "margin_m", "heartbeat_hz", "link_timeout_s"):
        if not (isinstance(s[k], (int, float)) and s[k] > 0):
            raise ValueError(f"scenario.mesh_dropout.{k} must be a positive number, got {s[k]!r}")
    arcs = s["heading_sectors_deg"]
    if arcs is not None and not (isinstance(arcs, list) and arcs and all(isinstance(a, (list, tuple)) and len(a) == 2 for a in arcs)):
        raise ValueError("scenario.mesh_dropout.heading_sectors_deg must be a list of [from, to] pairs in degrees")
    if not (isinstance(s["random_waypoints"], int) and s["random_waypoints"] >= 0):
        raise ValueError("scenario.mesh_dropout.random_waypoints must be an integer >= 0")
    return s


def _d3(p, q):
    return math.dist(p, q)


def plan(cfg):
    s = settings(cfg)
    agents = cfg["agents"]
    index = {a["name"]: i for i, a in enumerate(agents)}
    spawn = {a["name"]: [float(v) for v in a["spawn_pose"][:3]] for a in agents}
    alt = {n: s["altitude_base_m"] + s["altitude_step_m"] * i for n, i in index.items()}
    pos3 = lambda n, xy: (xy[0], xy[1], spawn[n][2] + alt[n])           # z above the ground: spawn height + altitude above arming point
    anchors = [n for n in index if n not in s["wanderers"]]
    D = linkmodel.dropout_range_m(cfg)
    if D is None:
        raise ValueError("the configured signal never reaches the receiver floor: there is no dropout to demonstrate")
    margin = float(s["margin_m"])
    rng = random.Random(int(s["seed"]))
    out = {"seed": int(s["seed"]), "dropout_range_m": D, "margin_m": margin, "settings": s,
           "altitude_m": alt, "spawn": {n: spawn[n] for n in index}, "anchors": anchors, "wanderers": {}}

    for w in s["wanderers"]:
        home = spawn[w][:2]
        a_pos = {a: pos3(a, spawn[a][:2]) for a in anchors}
        inside = lambda xy: all(_d3(pos3(w, xy), a_pos[a]) < D - margin for a in anchors)

        def random_point():
            for _ in range(1000):
                r, th = s["roam_radius_m"] * math.sqrt(rng.random()), rng.uniform(0, 2 * math.pi)
                p = [home[0] + r * math.cos(th), home[1] + r * math.sin(th)]
                if inside(p):
                    return [round(p[0], 2), round(p[1], 2)]
            raise ValueError(f"roam_radius_m {s['roam_radius_m']} leaves no point where every link to an anchor stays up (dropout range {D:.1f} m)")

        def draw_heading():
            arcs = s["heading_sectors_deg"]
            if not arcs:
                return rng.uniform(0, 360)
            lens = [(hi - lo) % 360 or 360 for lo, hi in arcs]
            lo, hi = arcs[rng.choices(range(len(arcs)), weights=lens)[0]]
            return (lo + rng.uniform(0, (hi - lo) % 360 or 360)) % 360

        k = int(rng.choice(list(s["excursion_links"])))
        target = None
        for _ in range(5000):
            th = math.radians(float(s["heading_deg"])) if s["heading_deg"] is not None else math.radians(draw_heading())
            feasible = []
            r = 5.0
            while r <= s["max_excursion_m"]:
                p = [home[0] + r * math.cos(th), home[1] + r * math.sin(th)]
                dist = {a: _d3(pos3(w, p), a_pos[a]) for a in anchors}
                dropped = [a for a in anchors if dist[a] > D + margin]
                kept = [a for a in anchors if dist[a] < D - margin]
                if len(dropped) == k and len(kept) == len(anchors) - k:
                    feasible.append((r, p, dist, dropped))
                elif len(dropped) > k:
                    break                                   # farther out only drops more
                r += 0.25
            if feasible:
                r, p, dist, dropped = feasible[len(feasible) // 2]       # middle of the feasible stretch: the largest margin
                target = {"xy": [round(p[0], 2), round(p[1], 2)], "heading_deg": round(math.degrees(th) % 360, 1), "distance_from_home_m": round(r, 1),
                          "k": k, "dropped_anchors": dropped, "kept_anchors": [a for a in anchors if a not in dropped],
                          "distance_to_anchors_m": {a: round(dist[a], 1) for a in anchors}}
                break
            if s["heading_deg"] is not None:
                raise ValueError(f"heading_deg {s['heading_deg']} cannot drop exactly {k} link(s) with margin {margin} m (anchor spread too small for that direction)")
        if target is None:
            raise ValueError(f"no excursion found that drops exactly {k} link(s) of {w} with margin {margin} m; try a smaller margin_m or other excursion_links")

        wp, labels, dwell = [], [], {}
        for _ in range(s["random_waypoints"]):
            wp.append(random_point()); labels.append("random")
        # the long legs run along the excursion heading: out through an outbound point just outside the roaming area, and back through it
        th = math.radians(target["heading_deg"])
        outb = [round(home[0] + (s["roam_radius_m"] + 3.0) * math.cos(th), 2), round(home[1] + (s["roam_radius_m"] + 3.0) * math.sin(th), 2)]
        if not inside(outb):
            raise ValueError("the outbound point of the excursion would already drop a link; use a smaller roam_radius_m")
        wp.append(outb); labels.append("outbound")
        wp.append(target["xy"]); labels.append("excursion"); dwell[len(wp) - 1] = float(s["dwell_s"])
        wp.append(outb); labels.append("outbound")
        for _ in range(s["random_waypoints"]):
            wp.append(random_point()); labels.append("random")
        wp.append([round(home[0], 2), round(home[1], 2)]); labels.append("home")
        length = sum(math.dist(a, b) for a, b in zip([home] + wp[:-1], wp))
        out["wanderers"][w] = {"waypoints_enu": wp, "labels": labels, "dwell_s": {str(i): d for i, d in dwell.items()},
                               "excursion": target, "path_length_m": round(length, 1),
                               "expected_flight_s": round(length / s["speed_mps"] + sum(dwell.values()), 1)}
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("config")
    ap.add_argument("--out")
    a = ap.parse_args()
    try:
        p = plan(yaml.safe_load(open(a.config)))
    except ValueError as e:
        sys.exit(f"mesh_plan: {e}")
    if a.out:
        json.dump(p, open(a.out, "w"), indent=1)
    print(json.dumps({k: v for k, v in p.items() if k != "settings"}, indent=1))


if __name__ == "__main__":
    main()
