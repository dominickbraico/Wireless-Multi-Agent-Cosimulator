"""Run-validity helpers shared by the analysis tools.

RTF criterion: the sliding mean of Gazebo's real-time factor over `window_s`
seconds must stay >= min_rtf for the whole measured window. Single 100 ms /stats samples are
noisy (isolated dips to ~0.25 were seen about once per 3-minute run, unrelated to any command),
so they are REPORTED but do not by themselves invalidate a run.
"""


def rtf_check(samples, t_start, t_end, min_rtf, window_s=5.0):
    """samples: [(t_wall, rtf)]. Returns dict(ok, worst_window_mean, worst_sample, n_below, n)."""
    # Only samples INSIDE the measured window: the one-off stall when the sweep client joins the
    # transport happens just before it and must not leak in through the sliding window.
    s = [(t, v) for t, v in samples if t_start <= t <= t_end]
    inwin = s
    if not inwin:
        return {"ok": False, "worst_window_mean": None, "worst_sample": None, "n_below": 0, "n": 0}
    worst = 1e9
    for t, _ in inwin:
        if t < t_start + window_s:                       # first window not yet full
            continue
        w = [v for tt, v in s if t - window_s <= tt <= t]
        if len(w) >= max(3, int(window_s * 5)):          # need a reasonably full window
            worst = min(worst, sum(w) / len(w))
    if worst == 1e9:                                     # window longer than the data: whole-run mean
        worst = sum(v for _, v in inwin) / len(inwin)
    return {"ok": worst >= min_rtf, "worst_window_mean": worst,
            "worst_sample": min(v for _, v in inwin),
            "n_below": sum(1 for _, v in inwin if v < min_rtf), "n": len(inwin)}


def rtf_from_poses(rows, node=0, step=5):
    """Real-time-factor samples [(t_wall, rtf)] for simulators with no stats topic (Isaac Sim), from the pose feed as ns-3 applied it
    (ns3/pose_applied.csv rows as dicts with `node`, `t_sim_s`, `t_wall_apply_s`).

    rtf = simulated seconds / wall seconds across `step` consecutive applied samples of one node (about 0.1 s at the 50 Hz default).
    The pose feed repeats the last pose when the simulator is blocked or slow, so t_sim stands still while t_wall advances and the factor falls to 0:
    a stall is counted, not hidden. Fewer than step + 1 samples give no samples. Assumption: t_sim is the simulator's own clock (Isaac: world.current_time)."""
    r = sorted(((float(x["t_wall_apply_s"]), float(x["t_sim_s"])) for x in rows if int(x["node"]) == node))
    out = []
    for i in range(step, len(r)):
        dw = r[i][0] - r[i - step][0]
        if dw > 0:
            out.append((r[i][0], (r[i][1] - r[i - step][1]) / dw))
    return out
