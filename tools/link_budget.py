#!/usr/bin/env python3
"""Predict RSSI / SNR / delivery vs distance for the configured Wi-Fi link BEFORE any sweep.

  tools/link_budget.py <experiment.yaml> [--per-bin PATH_TO_per_table_binary] [--out predictions.json]

Uses the same pieces ns-3 uses: LogDistance loss, kTB + noise-figure floor, and
the configured error-rate model (default TableBased, which is what YansWifiPhyHelper uses; via the per_table helper) for the data and ACK modes.
A frame is only DETECTED if rssi >= preamble_detection_min_rssi_dbm and snr >= preamble_detection_snr_db
(ns-3 ThresholdPreambleDetectionModel; v1 of this tool omitted this and predicted 82 m instead of 51 m).
Unicast delivery = at least one of the MAC attempts gets the data frame through
(ACK loss only costs retries, duplicates are filtered). Ping needs both directions.
Frame sizes: 1000 B UDP payload -> 1064 B MPDU; ACK 14 B.
"""
import argparse
import csv
import json
import math
import os
import subprocess
import sys

import yaml

HOME = os.path.expanduser("~" + os.environ["SUDO_USER"]) if os.environ.get("SUDO_USER") else os.path.expanduser("~")
DEFAULT_PER = f"{HOME}/dev/ns-3-build/ns-allinone-3.48/ns-3.48/build/scratch/per_table/ns3.48-per_table-optimized"
K_BOLTZ, T_REF, NOISE_FIGURE_DB = 1.3803e-23, 290.0, 7.0   # ns-3 defaults (RxNoiseFigure 7 dB)
BW_HZ = 20e6                                                # 802.11a
DATA_BYTES, ACK_BYTES, MAX_ATTEMPTS = 1064, 14, 7           # MaxSsrc default 7


def noise_dbm(nf_db=NOISE_FIGURE_DB):
    return 10 * math.log10(K_BOLTZ * T_REF * BW_HZ * 10 ** (nf_db / 10)) + 30


def load_per(cfg, binary):
    w = cfg["network"]["wifi"]
    out = subprocess.run([binary, w["data_mode"], w["control_mode"], str(DATA_BYTES), str(ACK_BYTES),
                          w.get("error_rate_model", "TableBased")],
                         capture_output=True, text=True, check=True).stdout
    rows = list(csv.DictReader(out.splitlines()))
    return [(float(r["snr_db"]), float(r["per_data"]), float(r["per_ack"])) for r in rows]


def interp(table, snr, col):
    if snr <= table[0][0]:
        return table[0][col]
    if snr >= table[-1][0]:
        return table[-1][col]
    for a, b in zip(table, table[1:]):
        if a[0] <= snr <= b[0]:
            f = (snr - a[0]) / (b[0] - a[0])
            return a[col] + f * (b[col] - a[col])


def predict(cfg, table, retry_limit=None):
    loss = cfg["network"]["channel"]["loss"][0]
    tx = cfg["network"]["wifi"]["tx_power_dbm"]
    phy = cfg["network"]["wifi"].get("phy", {})
    n = noise_dbm(phy.get("rx_noise_figure_db", NOISE_FIGURE_DB))
    min_rssi = phy.get("preamble_detection_min_rssi_dbm", -82.0)
    min_snr = phy.get("preamble_detection_snr_db", 4.0)
    attempts = retry_limit or MAX_ATTEMPTS
    rows = []
    d = 1.0
    while d <= 250.0:
        rssi = tx - loss["ref_loss_db"] - 10 * loss["exponent"] * math.log10(d / loss["ref_distance_m"])
        snr = rssi - n
        p_data_ok = 1 - interp(table, snr, 1)
        p_ack_ok = 1 - interp(table, snr, 2)
        detected = 1.0 if (rssi >= min_rssi and snr >= min_snr) else 0.0
        p_data_ok *= detected
        p_ack_ok *= detected
        deliver = 1 - (1 - p_data_ok) ** attempts
        rows.append({"d_m": d, "rssi_dbm": rssi, "snr_db": snr, "p_frame_ok": p_data_ok,
                     "p_ack_ok": p_ack_ok, "udp_loss": 1 - deliver, "ping_loss": 1 - deliver ** 2,
                     "attempts_mean": (1 - (1 - p_data_ok * p_ack_ok) ** attempts) / max(p_data_ok * p_ack_ok, 1e-12)})
        d += 0.5
    return rows, n


def limited_by(cfg, rows):
    """Which mechanism sets the 50 % point: the RSSI floor, the preamble SNR floor, or the PER curve."""
    phy = cfg["network"]["wifi"].get("phy", {})
    d50 = next(r for r in rows if r["udp_loss"] >= 0.5)
    prev = rows[max(0, rows.index(d50) - 1)]
    if prev["rssi_dbm"] >= phy.get("preamble_detection_min_rssi_dbm", -82.0) > d50["rssi_dbm"]:
        return "RSSI detection floor"
    if prev["snr_db"] >= phy.get("preamble_detection_snr_db", 4.0) > d50["snr_db"]:
        return "preamble SNR floor"
    return "packet error rate"


def first_distance(rows, key, thresh):
    for r in rows:
        if r[key] >= thresh:
            return r["d_m"]
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("config")
    ap.add_argument("--per-bin", default=DEFAULT_PER)
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    cfg = yaml.safe_load(open(a.config))
    rows, n = predict(cfg, load_per(cfg, a.per_bin), cfg["network"]["wifi"].get("retry_limit"))
    summ = {"noise_floor_dbm": round(n, 3),
            "d_udp_loss_1pct_m": first_distance(rows, "udp_loss", 0.01),
            "d_udp_loss_50pct_m": first_distance(rows, "udp_loss", 0.50),
            "d_udp_loss_99pct_m": first_distance(rows, "udp_loss", 0.99),
            "snr_at_50pct_db": next(r["snr_db"] for r in rows if r["udp_loss"] >= 0.5),
            "limited_by": limited_by(cfg, rows)}
    print(json.dumps(summ, indent=2))
    if a.out:
        json.dump({"summary": summ, "rows": rows}, open(a.out, "w"))


if __name__ == "__main__":
    main()
