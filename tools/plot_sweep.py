"""Plots for the distance sweep (called by analyze_sweep.py).

Three small multiples, one axis each (no dual axes), full range on the left and a zoom on the
cliff on the right: RSSI, loss, latency. Series colors are the validated categorical slots 1-3;
the model/prediction is neutral gray dashed (not a series hue). Forward pass = filled marker,
return pass = hollow marker. Marker shape also encodes the flow (aqua is < 3:1 on the light
surface, so shape + direct labels carry identity; the table view is sweep_stats.csv).
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import math
from matplotlib.ticker import FuncFormatter, NullFormatter

SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
MODEL = "#8a8985"


def make_plots(stats, pred, out_png):
    d50 = pred["summary"]["d_udp_loss_50pct_m"]
    d99 = pred["summary"]["d_udp_loss_99pct_m"]
    zoom = (max(0, d50 - 12), d50 + 12)
    prow = pred["rows"]
    plt.rcParams.update({"font.family": "DejaVu Sans", "text.color": INK, "axes.labelcolor": INK2,
                         "xtick.color": INK2, "ytick.color": INK2})
    fig, axes = plt.subplots(3, 2, figsize=(13, 9.5), facecolor=SURFACE,
                             gridspec_kw={"width_ratios": [2.2, 1], "hspace": 0.32, "wspace": 0.12})

    def series(key, scale=1.0, only=None):
        out = {"fwd": ([], []), "ret": ([], [])}
        for r in stats:
            v = r.get(key)
            if v is None or (only and not only(r)):
                continue
            out[r["pass"]][0].append(r["d_m"])
            out[r["pass"]][1].append(v * scale)
        return out

    def plot_series(ax, key, color, marker, label, scale=1.0):
        s = series(key, scale)
        ax.plot(*s["fwd"], color=color, marker=marker, ms=6.5, lw=1.5, label=label + " (out)", zorder=4)
        ax.plot(*s["ret"], color=color, marker=marker, ms=6.5, lw=0, mfc=SURFACE, mew=1.6,
                label=label + " (back)", zorder=5)

    def model_line(ax, key, scale=1.0, label=None):
        ax.plot([r["d_m"] for r in prow], [r[key] * scale for r in prow], color=MODEL, lw=1.5, ls=(0, (5, 3)),
                label=label, zorder=2)

    for row in range(3):
        for col in range(2):
            ax = axes[row][col]
            ax.set_facecolor(SURFACE)
            ax.grid(True, color=GRID, lw=0.8, zorder=0)
            for sp in ("top", "right"):
                ax.spines[sp].set_visible(False)
            for sp in ("left", "bottom"):
                ax.spines[sp].set_color(GRID)
            ax.axvline(d50, color=MODEL, lw=1, ls=":", zorder=1)
            ax.set_xlim((0, 210) if col == 0 else zoom)
            ax.axvspan(d99, 400, color=GRID, alpha=0.55, lw=0, zorder=0)
            if row == 2:
                ax.set_xlabel("distance between agents (m)")

    # row 0: RSSI
    for col in range(2):
        ax = axes[0][col]
        model_line(ax, "rssi_dbm", label="LogDistance model")
        s = series("rssi_med_dbm")
        ax.plot(*s["fwd"], color=BLUE, marker="o", ms=6.5, lw=0, label="measured (out)", zorder=4)
        ax.plot(*s["ret"], color=BLUE, marker="o", ms=6.5, lw=0, mfc=SURFACE, mew=1.6, label="measured (back)", zorder=5)
        ax.set_ylim(-105, -55)
    axes[0][0].set_ylabel("RSSI (dBm)")
    axes[0][0].set_title("Received signal strength", loc="left", fontsize=11, color=INK, fontweight="bold")
    axes[0][0].text(3, -103, f"dotted line: predicted 50 % loss at {d50:.1f} m   |   shaded: link dead (predicted 99 % loss)", color=INK2, fontsize=8.5, va="bottom")

    # row 1: loss %
    for col in range(2):
        ax = axes[1][col]
        model_line(ax, "udp_loss", 100, "model (UDP)")
        plot_series(ax, "udp_loss", BLUE, "o", "UDP 2 Mbit/s", 100)
        plot_series(ax, "ping_loss", ORANGE, "s", "ping", 100)
        plot_series(ax, "swarm_loss", AQUA, "^", "swarm 10 Hz", 100)
        ax.set_ylim(-4, 104)
    axes[1][0].set_ylabel("loss (%)")
    axes[1][0].set_title("Packet loss by flow", loc="left", fontsize=11, color=INK, fontweight="bold")

    # row 2: latency (log)
    for col in range(2):
        ax = axes[2][col]
        plot_series(ax, "rtt_p50_ms", ORANGE, "s", "ping RTT p50")
        plot_series(ax, "swarm_lat_p50_ms", AQUA, "^", "swarm latency p50")
        ax.set_yscale("log")
        ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:g}"))
        lo, hi = ax.get_ylim()
        # label minor ticks too when the axis spans < 1.5 decades (otherwise it has no labels at all)
        ax.yaxis.set_minor_formatter(FuncFormatter(lambda v, _: f"{v:g}") if math.log10(hi / lo) < 1.5 else NullFormatter())
    axes[2][0].set_ylabel("latency, median (ms, log)")
    axes[2][0].set_title("Latency of delivered packets", loc="left", fontsize=11, color=INK, fontweight="bold")

    axes[0][1].set_title(f"zoom on the cliff ({zoom[0]:.0f}-{zoom[1]:.0f} m)", loc="left", fontsize=11, color=INK2)
    for r in range(3):
        axes[r][1].set_ylabel("")

    # one legend per row on the full-range panel (>= 2 series always has a legend)
    for r in range(3):
        h, l = axes[r][0].get_legend_handles_labels()
        loc = {0: "upper right", 1: "upper left", 2: "lower right"}[r]
        axes[r][0].legend(h, l, loc=loc, fontsize=8.5, frameon=False, ncol=2 if r else 1, labelcolor=INK2)
    fig.suptitle("Distance sweep: link quality vs. agent separation (filled = moving away, hollow = coming back)",
                 x=0.06, ha="left", fontsize=12.5, color=INK, fontweight="bold", y=0.955)
    fig.savefig(out_png, dpi=150, facecolor=SURFACE, bbox_inches="tight")
    plt.close(fig)
