#!/usr/bin/env python3
"""Delivery ratio and latency of every radio topic, per ordered agent pair, from the relay logs of a `custom` scenario run.

    analyze_radio.py <log dir> [--min-delivery RATIO]      exit 1 if any pair is below RATIO (default: no limit)

Input: <log dir>/relay/relay_<agent>.csv, written by each agent's radio relay (ros2_ws/src/radio_relay):
    t_s,direction,topic,bytes,hash      direction tx = the agent sent this message to the radio, rx = it received it from the radio
The user's messages carry no sequence number or timestamp of ours, so sent and received messages are matched by content:
a tx row of agent A and an rx row of agent B are the same message when topic and payload hash are equal. Payloads that
repeat (a constant state) are matched in order: the k-th tx of a hash with the k-th rx of that hash, not earlier than the tx.

Definitions
  sent        tx rows of A on the topic, excluding the last TAIL_S seconds before A's final row (still in flight when the run stopped)
  received    how many of those B logged as rx
  delivery    received / sent
  latency     t_rx(B) - t_tx(A) for matched messages. Both clocks are the host's wall clock (the containers share it).
              It covers relay A -> radio -> relay B; the loopback hops between the user's node and its relay are outside it.
Limits: identical payloads from two different senders cannot be told apart at a receiver; each receiver row is used once,
by the first sender that claims it, so with identical concurrent payloads the per-pair figures are approximate.
Output: one line per topic and pair, and <log dir>/radio_report.json.
"""
import argparse
import csv
import glob
import json
import os
import sys
from collections import defaultdict

TAIL_S = 1.0


def read(log_dir):
    """-> {agent: {"tx": {topic: [(t, hash)]}, "rx": {topic: {hash: [t, ...]}}}}"""
    out = {}
    for path in sorted(glob.glob(os.path.join(log_dir, "relay", "relay_*.csv"))):
        agent = os.path.basename(path)[len("relay_"):-len(".csv")]
        tx, rx = defaultdict(list), defaultdict(lambda: defaultdict(list))
        with open(path) as f:
            for row in csv.DictReader(f):
                try:
                    t = float(row["t_s"])
                except (TypeError, ValueError):
                    continue                      # a row cut short when the container stopped
                if row["direction"] == "tx":
                    tx[row["topic"]].append((t, row["hash"]))
                elif row["direction"] == "rx":
                    rx[row["topic"]][row["hash"]].append(t)
        out[agent] = {"tx": tx, "rx": rx}
    return out


def percentile(values, q):
    if not values:
        return None
    v = sorted(values)
    return v[min(len(v) - 1, int(round(q * (len(v) - 1))))]


def analyze(logs):
    rows = []
    for a, la in logs.items():
        for topic, sent_all in la["tx"].items():
            if not sent_all:
                continue
            cutoff = sent_all[-1][0] - TAIL_S
            sent = [(t, h) for t, h in sent_all if t <= cutoff]
            for b, lb in logs.items():
                if b == a:
                    continue
                pool = {h: list(ts) for h, ts in lb["rx"].get(topic, {}).items()}     # receiver times per hash, consumed in order
                lat = []
                for t, h in sent:
                    ts = pool.get(h)
                    while ts and ts[0] < t - 0.005:        # an rx before this tx belongs to an earlier identical payload (5 ms clock slack)
                        ts.pop(0)
                    if ts:
                        lat.append((ts.pop(0) - t) * 1000.0)
                rows.append({"topic": topic, "tx": a, "rx": b, "sent": len(sent), "received": len(lat),
                             "delivery": (len(lat) / len(sent)) if sent else None,
                             "latency_ms_p50": percentile(lat, 0.5), "latency_ms_p99": percentile(lat, 0.99)})
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("log_dir")
    ap.add_argument("--min-delivery", type=float, default=None)
    a = ap.parse_args()
    logs = read(a.log_dir)
    if not logs:
        sys.exit(f"analyze_radio.py: no relay logs in {a.log_dir}/relay")
    rows = analyze(logs)
    bad = 0
    f = lambda v, n: "n/a" if v is None else f"{v:.{n}f}"
    for r in sorted(rows, key=lambda r: (r["topic"], r["tx"], r["rx"])):
        low = a.min_delivery is not None and (r["delivery"] is None or r["delivery"] < a.min_delivery)
        bad += low
        print(f"radio {r['topic']} {r['tx']}->{r['rx']}: sent={r['sent']} received={r['received']} delivery={f(r['delivery'], 3)} "
              f"latency_ms p50={f(r['latency_ms_p50'], 2)} p99={f(r['latency_ms_p99'], 2)}" + ("   BELOW --min-delivery" if low else ""))
    json.dump({"tail_excluded_s": TAIL_S, "pairs": rows}, open(os.path.join(a.log_dir, "radio_report.json"), "w"), indent=1)
    if not rows:
        sys.exit("analyze_radio.py: the relays sent nothing")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
