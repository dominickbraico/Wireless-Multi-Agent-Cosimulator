#!/usr/bin/env python3
"""Flight plans of the mesh-dropout scenario: seeded random flights that make links between agents drop and recover.

    mesh_plan.py <experiment.yaml> [--out plan.json]      print (and optionally save) the plan the YAML produces

The plan is a pure function of the YAML (including the seed), so every agent container computes the same plan and a run can be repeated exactly.
Two modes (`scenario.mesh_dropout.mode`):

mode: roam (default)  EVERY agent flies. Each draws its own seeded random waypoints inside `arena_radius_m` of the centre of the spawn points, flies them
  at `speed_mps` and returns to its spawn point; all agents start together (a shared start signal after take-off) and stop together. Because everyone
  moves, whether a link is up depends on where both ends are at that moment, so the planner simulates the whole flight (a vehicle follows its
  setpoint about `tracking_lag_s` late) and keeps a plan only if (1) never more than `max_dropped_links` links are at risk at once (a link is at risk
  when its distance exceeds D - margin_m), and (2) at least `min_drop_events` links clearly drop (distance above D + margin_m for at least `min_drop_s`).
  Candidate plans are drawn from the seed until one qualifies. Distinct altitudes keep paths apart.

mode: excursion  ONE wanderer (or more) flies, the others hover at their spawn points (the anchors). The wanderer flies random points where every link is up,
  an excursion point where exactly k anchor links (k = 1 or 2) end beyond the dropout range, then back and home. See the previous behaviour in docs/MESH_DROPOUT.md.

D is the distance at which the YAML's own signal model reaches the receiver floor (orchestrator/linkmodel.py), taken in 3-D because the agents fly at different
altitudes (agent i flies at altitude_base_m + i * altitude_step_m above its arming point). `heading_sectors_deg` restricts where the flights may go: in roam mode
waypoints (and the straight legs between them) beyond `roam_radius_m + 3` m of the centre must lie in these polar arcs (ENU degrees, 0 = east, 90 = north); in excursion mode the
excursion heading is drawn from them.

Modelling notes: the margin has to cover the tracking error (about speed x tracking_lag_s while moving). Ground is assumed flat for the altitude arithmetic, as elsewhere.
Waypoints are ENU world metres.
"""
import argparse
import json
import math
import random
import sys

import yaml

import linkmodel

DEFAULTS = {
    "mode": "roam",               # roam: every agent flies | excursion: one wanderer, the rest hover
    "seed": 1,
    "arena_radius_m": 40.0,       # roam: waypoints are drawn within this radius of the centre of the spawn points
    "max_dropped_links": 2,       # roam: never more than this many links at risk of being down at once
    "min_drop_events": 2,         # roam: at least this many clear drops (distance > D + margin for min_drop_s)
    "min_drop_s": 6.0,
    "min_isolation_s": 0.0,       # roam: if > 0, at least one agent must have EVERY link below the floor (distance > D + margin to all others) this long;
                                  #       needs max_dropped_links >= number of agents - 1
    "min_flight_s": 70.0,         # roam: each agent's path is at least this long (seconds at speed_mps)
    "end_hold_s": 5.0,            # roam: everyone hovers at home this long before landing
    "tracking_lag_s": 1.0,        # roam: how late a vehicle follows its setpoint (measured: about 1 s)
    "max_attempts": 50000,        # roam: candidate plans tried before giving up
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
    if s["mode"] not in ("roam", "excursion"):
        raise ValueError(f"scenario.mesh_dropout.mode must be roam or excursion, got {s['mode']!r}")
    if s["mode"] == "excursion":
        if s["wanderers"] is None:
            s["wanderers"] = [names[-1]]
        bad = [w for w in s["wanderers"] if w not in names]
        if bad:
            raise ValueError(f"scenario.mesh_dropout.wanderers: unknown agents {bad}")
        if len(s["wanderers"]) >= len(names):
            raise ValueError("mode excursion: at least one agent must stay behind as an anchor (use mode roam to move every agent)")
        if not s["excursion_links"] or any(int(k) < 1 or int(k) > len(names) - len(s["wanderers"]) for k in s["excursion_links"]):
            raise ValueError(f"excursion_links {s['excursion_links']} must be between 1 and the number of anchors ({len(names) - len(s['wanderers'])})")
    else:
        if int(s["max_dropped_links"]) < 1 or int(s["min_drop_events"]) < 1:
            raise ValueError("max_dropped_links and min_drop_events must be at least 1")
    for k in ("speed_mps", "roam_radius_m", "dwell_s", "margin_m", "heartbeat_hz", "link_timeout_s", "arena_radius_m", "min_drop_s", "min_flight_s", "tracking_lag_s"):
        if not (isinstance(s[k], (int, float)) and s[k] > 0):
            raise ValueError(f"scenario.mesh_dropout.{k} must be a positive number, got {s[k]!r}")
    if s["mode"] == "roam" and float(s["min_isolation_s"]) > 0 and int(s["max_dropped_links"]) < len(names) - 1:
        raise ValueError(f"min_isolation_s needs max_dropped_links >= {len(names) - 1} (all of one agent's links are down at once)")
    arcs = s["heading_sectors_deg"]
    if arcs is not None and not (isinstance(arcs, list) and arcs and all(isinstance(a, (list, tuple)) and len(a) == 2 for a in arcs)):
        raise ValueError("scenario.mesh_dropout.heading_sectors_deg must be a list of [from, to] pairs in degrees")
    if not (isinstance(s["random_waypoints"], int) and s["random_waypoints"] >= 0):
        raise ValueError("scenario.mesh_dropout.random_waypoints must be an integer >= 0")
    return s


def _d3(p, q):
    return math.dist(p, q)


def _plan_excursion(cfg, s):
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
    out = {"mode": "excursion", "seed": int(s["seed"]), "dropout_range_m": D, "margin_m": margin, "settings": s,
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


def _plan_roam(cfg, s):
    agents = cfg["agents"]
    names = [a["name"] for a in agents]
    spawn = {a["name"]: [float(v) for v in a["spawn_pose"][:3]] for a in agents}
    alt = {n: s["altitude_base_m"] + s["altitude_step_m"] * i for i, n in enumerate(names)}
    D = linkmodel.dropout_range_m(cfg)
    if D is None:
        raise ValueError("the configured signal never reaches the receiver floor: there is no dropout to demonstrate")
    margin, speed, lag = float(s["margin_m"]), float(s["speed_mps"]), float(s["tracking_lag_s"])
    rng = random.Random(int(s["seed"]))
    centre = (sum(spawn[n][0] for n in names) / len(names), sum(spawn[n][1] for n in names) / len(names))
    inner, outer = float(s["roam_radius_m"]) + 3.0, float(s["arena_radius_m"])
    arcs = s["heading_sectors_deg"]

    def allowed(p):
        r = math.hypot(p[0] - centre[0], p[1] - centre[1])
        if r <= inner:
            return True
        if r > outer:
            return False
        if not arcs:
            return True
        th = math.degrees(math.atan2(p[1] - centre[1], p[0] - centre[0])) % 360
        return any((th - lo) % 360 <= ((hi - lo) % 360 or 360) for lo, hi in arcs)

    def leg_ok(a, b):                                   # the whole straight leg must stay where flying is allowed
        n = max(1, int(math.dist(a, b)))
        return all(allowed((a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n)) for i in range(n + 1))

    def draw_point():
        for _ in range(10000):
            r, th = outer * math.sqrt(rng.random()), rng.uniform(0, 2 * math.pi)
            p = (round(centre[0] + r * math.cos(th), 2), round(centre[1] + r * math.sin(th), 2))
            if allowed(p):
                return p
        raise ValueError("arena_radius_m / heading_sectors_deg leave no place to fly")

    def draw_path(home):
        for _ in range(1000):
            pts, length = [home], 0.0
            while True:
                p = draw_point()
                if not leg_ok(pts[-1], p):
                    continue
                pts.append(p); length += math.dist(pts[-2], p)
                back = math.dist(p, home)
                if (length + back) / speed >= s["min_flight_s"] and leg_ok(p, home):
                    pts.append(home)
                    times = [0.0]
                    for u, v in zip(pts, pts[1:]):
                        times.append(times[-1] + math.dist(u, v) / speed)
                    return pts, times
                if len(pts) > 40:
                    break
        raise ValueError("could not draw a flight path of min_flight_s inside the allowed area")

    def at(path, t):                                    # setpoint position at time t (clamped), linear along the legs
        pts, times = path
        if t <= 0:
            return pts[0]
        if t >= times[-1]:
            return pts[-1]
        i = max(j for j in range(len(times)) if times[j] <= t)
        f = (t - times[i]) / (times[i + 1] - times[i])
        return (pts[i][0] + f * (pts[i + 1][0] - pts[i][0]), pts[i][1] + f * (pts[i + 1][1] - pts[i][1]))

    def evaluate(paths, t_total, dt=0.25):
        pairs = [(a, b) for i, a in enumerate(names) for b in names[i + 1:]]
        runs = {p: None for p in pairs}
        events, peak, max_risk = [], {p: 0.0 for p in pairs}, 0
        iso_run, isolations = {n: None for n in names}, []
        t = 0.0
        while t <= t_total + 1e-9:
            xy = {n: at(paths[n], t - lag) for n in names}
            risk = 0
            far = {n: True for n in names}                  # n is beyond D + margin from every other agent
            for a, b in pairs:
                d = math.dist((xy[a][0], xy[a][1], spawn[a][2] + alt[a]), (xy[b][0], xy[b][1], spawn[b][2] + alt[b]))
                risk += d > D - margin
                if d <= D + margin:
                    far[a] = far[b] = False
                if d > D + margin:
                    runs[(a, b)] = (runs[(a, b)] or [t, 0.0])
                    peak[(a, b)] = max(peak[(a, b)], d)
                elif runs[(a, b)] is not None:
                    t0 = runs[(a, b)][0]
                    if t - t0 >= s["min_drop_s"]:
                        events.append({"pair": [a, b], "t_start_s": round(t0, 2), "t_end_s": round(t, 2), "peak_distance_m": round(peak[(a, b)], 1)})
                    runs[(a, b)] = None; peak[(a, b)] = 0.0
            max_risk = max(max_risk, risk)
            for n in names:
                if far[n]:
                    iso_run[n] = iso_run[n] if iso_run[n] is not None else t
                elif iso_run[n] is not None:
                    if t - iso_run[n] >= s["min_isolation_s"] > 0:
                        isolations.append({"agent": n, "t_start_s": round(iso_run[n], 2), "t_end_s": round(t, 2)})
                    iso_run[n] = None
            t += dt
        for n in names:                                     # an isolation still running at the end of the plan (the agents are home by then, so rare)
            if iso_run[n] is not None and t - iso_run[n] >= s["min_isolation_s"] > 0:
                isolations.append({"agent": n, "t_start_s": round(iso_run[n], 2), "t_end_s": round(t, 2)})
        return max_risk, sorted(events, key=lambda e: e["t_start_s"]), sorted(isolations, key=lambda e: e["t_start_s"])

    for attempt in range(1, int(s["max_attempts"]) + 1):
        paths = {n: draw_path((round(spawn[n][0], 2), round(spawn[n][1], 2))) for n in names}
        t_total = max(p[1][-1] for p in paths.values()) + float(s["end_hold_s"])
        max_risk, events, isolations = evaluate(paths, t_total)
        if (max_risk <= int(s["max_dropped_links"]) and len(events) >= int(s["min_drop_events"])
                and (s["min_isolation_s"] <= 0 or isolations)):
            break
    else:
        raise ValueError(f"no plan with at most {s['max_dropped_links']} links at risk and {s['min_drop_events']} clear drops in {s['max_attempts']} attempts; "
                         "try a larger arena_radius_m, fewer min_drop_events or a smaller margin_m")
    return {"mode": "roam", "seed": int(s["seed"]), "dropout_range_m": D, "margin_m": margin, "settings": s, "altitude_m": alt,
            "spawn": {n: spawn[n] for n in names}, "centre": [centre[0], centre[1]], "attempts": attempt,
            "t_total_s": round(t_total, 2), "max_links_at_risk": max_risk, "nominal_events": events, "nominal_isolations": isolations,
            "agents": {n: {"waypoints_enu": [list(p) for p in paths[n][0]], "times_s": [round(t, 2) for t in paths[n][1]],
                           "path_length_m": round(sum(math.dist(u, v) for u, v in zip(paths[n][0], paths[n][0][1:])), 1)} for n in names}}


def plan(cfg):
    s = settings(cfg)
    return _plan_roam(cfg, s) if s["mode"] == "roam" else _plan_excursion(cfg, s)


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
