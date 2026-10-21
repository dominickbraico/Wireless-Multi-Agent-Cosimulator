#!/usr/bin/env bash
# Offline checks of the mesh-dropout flight plan (orchestrator/mesh_plan.py). No Docker, no ROS, no simulator: a few seconds.
#   tests/mesh_plan.sh [--config YAML]      (default: four_agent_mesh_dropout.yaml)
# For 40 seeds: the plan is repeatable, the excursion drops exactly the planned number of anchor links with the planned margin, every random
# waypoint keeps all links up, the path ends at the spawn point, and different seeds give different plans.
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CFG="$ROOT/config/experiments/four_agent_mesh_dropout.yaml"
[ "${1:-}" = "--config" ] && CFG="$2"
python3 - "$ROOT" "$CFG" <<'PY'
import copy, math, sys, yaml
root, cfgp = sys.argv[1:3]
sys.path.insert(0, root + "/orchestrator")
import linkmodel, mesh_plan
cfg = yaml.safe_load(open(cfgp))
ok = True
def check(name, cond, detail=""):
    global ok
    ok &= bool(cond)
    print(("PASS  " if cond else "FAIL  ") + name + (f"  ({detail})" if detail else ""))

D = linkmodel.dropout_range_m(cfg)
def with_seed(s):
    c = copy.deepcopy(cfg); c.setdefault("scenario", {}).setdefault("mesh_dropout", {})["seed"] = s; return c

p1, p2 = mesh_plan.plan(with_seed(7)), mesh_plan.plan(with_seed(7))
check("same seed gives an identical plan", p1 == p2)
sigs = {str(mesh_plan.plan(with_seed(s))["wanderers"]) for s in range(40)}
check("different seeds give different plans", len(sigs) > 30, f"{len(sigs)} distinct of 40")

bad = []; ks = set(); worst_margin = 1e9
for seed in range(40):
    p = mesh_plan.plan(with_seed(seed)); m = p["margin_m"]
    z = lambda n, xy: (xy[0], xy[1], p["spawn"][n][2] + p["altitude_m"][n])
    for w, wp in p["wanderers"].items():
        d = lambda xy, a: math.dist(z(w, xy), z(a, p["spawn"][a][:2]))
        ex = wp["excursion"]; ks.add(ex["k"])
        for lab, xy in zip(wp["labels"], wp["waypoints_enu"]):
            dist = {a: d(xy, a) for a in p["anchors"]}
            if lab in ("random", "outbound") and not all(v < D - m for v in dist.values()):
                bad.append((seed, "random point drops a link", xy))
            if lab == "excursion":
                dropped = [a for a, v in dist.items() if v > D + m]
                kept = [a for a, v in dist.items() if v < D - m]
                if sorted(dropped) != sorted(ex["dropped_anchors"]) or len(dropped) != ex["k"] or len(kept) != len(p["anchors"]) - ex["k"]:
                    bad.append((seed, "excursion drops the wrong links", dist))
                worst_margin = min(worst_margin, min(abs(v - D) for v in dist.values()))
        arcs = p["settings"].get("heading_sectors_deg")
        if arcs and not any((ex["heading_deg"] - lo) % 360 <= (hi - lo) % 360 for lo, hi in arcs):
            bad.append((seed, "excursion heading outside the allowed sectors", ex["heading_deg"]))
        if wp["labels"].count("outbound") != 2:
            bad.append((seed, "excursion not routed through the outbound point"))
        if wp["waypoints_enu"][-1] != [round(p["spawn"][w][0], 2), round(p["spawn"][w][1], 2)] or wp["labels"][-1] != "home":
            bad.append((seed, "does not end at spawn"))
check("every random/outbound waypoint keeps all links up; headings stay inside the allowed sectors; every excursion drops exactly k links; every path ends at the spawn point", not bad, str(bad[:2]))
check("both 1-link and 2-link excursions occur over the seeds", ks == {1, 2}, f"k values {sorted(ks)}")
check(f"excursion margin at least the configured one in every plan", worst_margin >= 3.0 - 1e-6, f"smallest |distance - D| over 40 seeds {worst_margin:.2f} m, D = {D:.1f} m")
for badcfg, label in (({"wanderers": ["nobody"]}, "unknown wanderer"), ({"excursion_links": [9]}, "too many links"), ({"speed_mps": -1}, "negative speed"), ({"heading_deg": 0, "margin_m": 40}, "impossible margin")):
    c = copy.deepcopy(cfg); c.setdefault("scenario", {}).setdefault("mesh_dropout", {}).update(badcfg)
    try:
        mesh_plan.plan(c); check(f"rejects {label}", False, "no error")
    except ValueError:
        check(f"rejects {label}", True)
print("ALL PASS" if ok else "SOME FAILED"); sys.exit(0 if ok else 1)
PY
