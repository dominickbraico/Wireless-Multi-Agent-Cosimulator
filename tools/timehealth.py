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
