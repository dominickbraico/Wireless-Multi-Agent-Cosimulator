#!/usr/bin/env bash
# Offline checks of the mesh-dropout flight plans (orchestrator/mesh_plan.py). No Docker, no ROS, no simulator: seconds.
#   tests/mesh_plan.sh [--config YAML]      (default: four_agent_mesh_dropout.yaml; run it for the Baylands YAML too)
# Both modes, 40 seeds each.
#   roam      (every agent flies): plans are repeatable and differ between seeds; the flight is re-simulated here from the plan file alone (independently of the
#             planner): never more than max_dropped_links links at risk at once, at least min_drop_events clear drops, every path starts and ends at its spawn,
#             lasts at least min_flight_s at speed_mps, stays inside the allowed area (arena and heading sectors), and the agents' altitudes differ
#   excursion (one wanderer): the excursion drops exactly the planned number of anchor links with the planned margin, random points keep all links up, the path
#             ends at the spawn point, headings stay in the allowed sectors
# plus: bad YAML values are rejected.
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CFG="$ROOT/config/experiments/four_agent_mesh_dropout.yaml"
[ "${1:-}" = "--config" ] && CFG="$2"
python3 - "$ROOT" "$CFG" <<'PY'
import copy, math, sys, yaml
root, cfgp = sys.argv[1:3]
sys.path.insert(0, root + "/orchestrator")
import linkmodel, mesh_plan
base = yaml.safe_load(open(cfgp))
ok = True
def check(name, cond, detail=""):
    global ok
    ok &= bool(cond)
    print(("PASS  " if cond else "FAIL  ") + name + (f"  ({detail})" if detail else ""))
def variant(mode=None, seed=None, **kw):
    c = copy.deepcopy(base); m = c.setdefault("scenario", {}).setdefault("mesh_dropout", {})
    if mode: m["mode"] = mode
    if seed is not None: m["seed"] = seed
    m.update(kw); return c
D = linkmodel.dropout_range_m(base)

# ------------------------------------------------------------------ roam
print("-- mode roam")
p1, p2 = mesh_plan.plan(variant("roam", 7)), mesh_plan.plan(variant("roam", 7))
check("same seed gives an identical plan", p1 == p2)
check("different seeds give different plans", len({str(mesh_plan.plan(variant("roam", s))["agents"]) for s in range(40)}) == 40, "40 distinct of 40")

def pos(pl, n, t):
    a = pl["agents"][n]; ts, ws = a["times_s"], a["waypoints_enu"]
    if t <= 0: return ws[0]
    if t >= ts[-1]: return ws[-1]
    i = max(j for j in range(len(ts)) if ts[j] <= t); f = (t - ts[i]) / (ts[i + 1] - ts[i])
    return [ws[i][0] + f * (ws[i + 1][0] - ws[i][0]), ws[i][1] + f * (ws[i + 1][1] - ws[i][1])]

bad, attempts, nev = [], [], []
for seed in range(40):
    pl = mesh_plan.plan(variant("roam", seed)); st = pl["settings"]; names = list(pl["agents"]); attempts.append(pl["attempts"])
    m, lag, speed = pl["margin_m"], st["tracking_lag_s"], st["speed_mps"]
    # independent re-simulation from the plan file's waypoints and times
    pairs = [(a, b) for i, a in enumerate(names) for b in names[i + 1:]]
    run = {pq: None for pq in pairs}; clear = 0; max_risk = 0; t = 0.0
    while t <= pl["t_total_s"] + 1e-9:
        xy = {n: pos(pl, n, t - lag) for n in names}; risk = 0
        for a, b in pairs:
            d = math.dist((*xy[a], pl["spawn"][a][2] + pl["altitude_m"][a]), (*xy[b], pl["spawn"][b][2] + pl["altitude_m"][b]))
            risk += d > D - m
            if d > D + m:
                run[(a, b)] = run[(a, b)] if run[(a, b)] is not None else t
            elif run[(a, b)] is not None:
                clear += (t - run[(a, b)]) >= st["min_drop_s"]; run[(a, b)] = None
        max_risk = max(max_risk, risk); t += 0.25
    nev.append(clear)
    if max_risk > st["max_dropped_links"]: bad.append((seed, "too many links at risk", max_risk))
    if clear < st["min_drop_events"]: bad.append((seed, "too few clear drops", clear))
    c = pl["centre"]; inner, outer = st["roam_radius_m"] + 3.0, st["arena_radius_m"]; arcs = st.get("heading_sectors_deg")
    def allowed(q):
        r = math.hypot(q[0] - c[0], q[1] - c[1])
        if r <= inner: return True
        if r > outer + 1e-6: return False
        if not arcs: return True
        th = math.degrees(math.atan2(q[1] - c[1], q[0] - c[0])) % 360
        return any((th - lo) % 360 <= ((hi - lo) % 360 or 360) for lo, hi in arcs)
    for n in names:
        a = pl["agents"][n]; ws, ts = a["waypoints_enu"], a["times_s"]
        sp = [round(pl["spawn"][n][0], 2), round(pl["spawn"][n][1], 2)]
        if ws[0] != sp or ws[-1] != sp: bad.append((seed, n, "does not start and end at its spawn"))
        if ts[-1] < st["min_flight_s"] - 1e-6: bad.append((seed, n, "path shorter than min_flight_s", ts[-1]))
        if pl["t_total_s"] < ts[-1] + st["end_hold_s"] - 1e-6: bad.append((seed, n, "end time too early"))
        for (u, v), (t0_, t1_) in zip(zip(ws, ws[1:]), zip(ts, ts[1:])):
            if abs(math.dist(u, v) / (t1_ - t0_) - speed) > 0.05: bad.append((seed, n, "leg not at speed_mps")); break
            k = max(1, int(math.dist(u, v)))
            if not all(allowed((u[0] + (v[0] - u[0]) * i / k, u[1] + (v[1] - u[1]) * i / k)) for i in range(k + 1)): bad.append((seed, n, "leg leaves the allowed area", u, v)); break
    if len(set(pl["altitude_m"].values())) != len(names): bad.append((seed, "altitudes not distinct"))
check("re-simulated: at most max_dropped_links links at risk at once, at least min_drop_events clear drops, paths start/end at spawn, at speed, inside the allowed area", not bad, str(bad[:2]))
check("planner needs few candidates", max(attempts) < 5000, f"max {max(attempts)} candidate plans over 40 seeds, median {sorted(attempts)[20]}; clear drops per plan {min(nev)}..{max(nev)}")

# ------------------------------------------------------------------ excursion
print("-- mode excursion")
x1, x2 = mesh_plan.plan(variant("excursion", 7)), mesh_plan.plan(variant("excursion", 7))
check("same seed gives an identical plan", x1 == x2)
check("different seeds give different plans", len({str(mesh_plan.plan(variant("excursion", s))["wanderers"]) for s in range(40)}) > 30)
bad = []; ks = set(); worst_margin = 1e9
for seed in range(40):
    p = mesh_plan.plan(variant("excursion", seed)); m = p["margin_m"]
    z = lambda n, xy: (xy[0], xy[1], p["spawn"][n][2] + p["altitude_m"][n])
    for w, wp in p["wanderers"].items():
        d = lambda xy, a: math.dist(z(w, xy), z(a, p["spawn"][a][:2]))
        ex = wp["excursion"]; ks.add(ex["k"])
        for lab, xy in zip(wp["labels"], wp["waypoints_enu"]):
            dist = {a: d(xy, a) for a in p["anchors"]}
            if lab in ("random", "outbound") and not all(v < D - m for v in dist.values()): bad.append((seed, "a random/outbound point drops a link", xy))
            if lab == "excursion":
                dropped = [a for a, v in dist.items() if v > D + m]; kept = [a for a, v in dist.items() if v < D - m]
                if sorted(dropped) != sorted(ex["dropped_anchors"]) or len(dropped) != ex["k"] or len(kept) != len(p["anchors"]) - ex["k"]: bad.append((seed, "excursion drops the wrong links", dist))
                worst_margin = min(worst_margin, min(abs(v - D) for v in dist.values()))
        arcs = p["settings"].get("heading_sectors_deg")
        if arcs and not any((ex["heading_deg"] - lo) % 360 <= (hi - lo) % 360 for lo, hi in arcs): bad.append((seed, "heading outside the allowed sectors", ex["heading_deg"]))
        if wp["labels"].count("outbound") != 2: bad.append((seed, "excursion not routed through the outbound point"))
        if wp["waypoints_enu"][-1] != [round(p["spawn"][w][0], 2), round(p["spawn"][w][1], 2)] or wp["labels"][-1] != "home": bad.append((seed, "does not end at spawn"))
check("every random/outbound waypoint keeps all links up; headings stay inside the allowed sectors; every excursion drops exactly k links; every path ends at the spawn point", not bad, str(bad[:2]))
check("both 1-link and 2-link excursions occur over the seeds", ks == {1, 2}, f"k values {sorted(ks)}")
check("excursion margin at least the configured one in every plan", worst_margin >= 3.0 - 1e-6, f"smallest |distance - D| over 40 seeds {worst_margin:.2f} m, D = {D:.1f} m")

# ------------------------------------------------------------------ rejected input
print("-- bad input")
for kw, label in (({"mode": "excursion", "wanderers": ["nobody"]}, "unknown wanderer"), ({"mode": "excursion", "excursion_links": [9]}, "too many links"), ({"speed_mps": -1}, "negative speed"),
                  ({"mode": "excursion", "heading_deg": 0, "margin_m": 40}, "impossible margin"), ({"mode": "fly"}, "unknown mode"),
                  ({"mode": "roam", "min_drop_events": 500, "max_attempts": 50}, "impossible roam constraints")):
    try:
        mesh_plan.plan(variant(**kw)); check(f"rejects {label}", False, "no error")
    except ValueError:
        check(f"rejects {label}", True)
print("ALL PASS" if ok else "SOME FAILED"); sys.exit(0 if ok else 1)
PY
