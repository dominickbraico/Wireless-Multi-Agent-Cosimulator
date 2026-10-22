#!/usr/bin/env python3
"""The signal model the experiment YAML describes, evaluated in Python: the same arithmetic ns-3 does for the channel.

Used by the map (tools/plot_comm_map.py) when a run has no ns-3 pair log, and by the mesh-dropout planner
(orchestrator/mesh_plan.py) to place waypoints where a link is predicted to drop. Derivation: docs/SIGNAL_MODEL.md.

    rx_power_dbm(cfg, d)      received power in dBm at antenna distance d (m): tx power minus the chained LogDistance losses
    floor_dbm(cfg)            receiver preamble-detection RSSI floor: a frame weaker than this is never detected
    dropout_range_m(cfg)      distance at which the modelled signal reaches the floor
"""
import math


def tx_power_dbm(cfg):
    return float(cfg["network"]["wifi"]["tx_power_dbm"])


def rx_power_dbm(cfg, d):
    """Received power at distance d (m) after the configured chain of LogDistance losses, as ns-3 computes it."""
    p = tx_power_dbm(cfg)
    for m in cfg["network"]["channel"]["loss"]:
        if m["type"] != "LogDistance":
            raise SystemExit(f"loss model {m['type']} is not supported by the Python link model (only LogDistance)")
        # ns-3 3.48 LogDistancePropagationLossModel::DoCalcRxPower: inside the reference distance only the reference loss applies
        p -= m["ref_loss_db"]
        if d > m["ref_distance_m"]:
            p -= 10.0 * m["exponent"] * math.log10(d / m["ref_distance_m"])
    return p


def floor_dbm(cfg):
    return float(cfg["network"]["wifi"].get("phy", {}).get("preamble_detection_min_rssi_dbm", -82.0))


def dropout_range_m(cfg, floor=None):
    """Distance (m) at which the modelled signal falls to the dropout floor, by bisection over the same chain of losses
    (received power only decreases with distance for LogDistance). None if it does not reach the floor within 100 km."""
    floor = floor_dbm(cfg) if floor is None else floor
    lo, hi = 1e-3, 1e5
    if rx_power_dbm(cfg, hi) >= floor:
        return None
    for _ in range(80):
        mid = (lo * hi) ** 0.5
        lo, hi = (mid, hi) if rx_power_dbm(cfg, mid) >= floor else (lo, mid)
    return hi
