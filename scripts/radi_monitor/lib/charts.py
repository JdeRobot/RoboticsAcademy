"""
charts.py — Matplotlib chart generation for radi_monitor.

All public functions return a base64-encoded PNG string suitable for inline
embedding in HTML (<img src="data:image/png;base64,...">).
"""

import base64
import io
import sys

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("matplotlib not found. Run: pip3 install matplotlib")

from .analysis import category_cpu_over_time
from .collector import CATEGORY_COLOR

_EXC_PALETTE = [
    "#1565c0",
    "#2e7d32",
    "#6a1b9a",
    "#c62828",
    "#00695c",
    "#ad1457",
    "#0277bd",
    "#558b2f",
    "#4e342e",
    "#37474f",
]

_CHART_RC = {
    "font.size": 11,
    "axes.grid": True,
    "grid.alpha": 0.35,
    "figure.facecolor": "white",
    "axes.facecolor": "#fafafa",
    "axes.spines.top": False,
    "axes.spines.right": False,
}


def _chart_setup():
    """Reset matplotlib state and apply project-wide rc params."""
    plt.rcdefaults()
    plt.rcParams.update(_CHART_RC)


def _fig_to_b64(fig):
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=110, bbox_inches="tight", facecolor="white")
    buf.seek(0)
    b64 = base64.b64encode(buf.read()).decode()
    plt.close(fig)
    return b64


def _add_exercise_bands(ax, exercise_events, times, ymax):
    for i, (t0, name, *_) in enumerate(exercise_events):
        t1 = exercise_events[i + 1][0] if i + 1 < len(exercise_events) else times[-1]
        color = _EXC_PALETTE[i % len(_EXC_PALETTE)]
        ax.axvspan(t0, t1, alpha=0.09, color=color, zorder=0)
        if name != "idle":
            label = name.replace("_harmonic", "").replace("_", " ")
            ax.text(
                (t0 + t1) / 2,
                ymax * 0.98,
                label,
                ha="center",
                va="top",
                fontsize=8,
                color=color,
                fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.2", facecolor="white", edgecolor=color, alpha=0.85),
            )


def chart_cpu(samples, exercise_events, cpu_threads):
    _chart_setup()
    fig, ax = plt.subplots(figsize=(14, 4))

    times = [s["t"] for s in samples]
    cont_cpu = [s["container"]["cpu_pct"] for s in samples]
    host_cpu = [s["host"]["cpu_pct"] for s in samples]
    ymax = max(max(cont_cpu), max(host_cpu), 10) * 1.18

    _add_exercise_bands(ax, exercise_events, times, ymax)
    ax.plot(times, cont_cpu, color="#d32f2f", linewidth=2, label="Container CPU %", zorder=3)
    ax.plot(
        times,
        host_cpu,
        color="#555",
        linewidth=1,
        linestyle="--",
        alpha=0.5,
        label="Host CPU %",
        zorder=2,
    )
    ax.fill_between(times, cont_cpu, alpha=0.07, color="#d32f2f")
    ax.axhline(
        y=cpu_threads * 100,
        color="#d32f2f",
        linewidth=0.8,
        linestyle=":",
        alpha=0.4,
        label=f"Max ({cpu_threads * 100}%)",
    )
    ax.set_ylim(0, ymax)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("CPU %")
    ax.set_title("Container CPU over time", fontweight="bold", fontsize=13)
    ax.legend(fontsize=9, loc="upper left")
    fig.tight_layout()
    return _fig_to_b64(fig)


def chart_rtf(samples, exercise_events):
    rtf_pts = [(s["t"], s["rtf"]) for s in samples if s.get("rtf") is not None]
    if not rtf_pts:
        return None

    _chart_setup()
    fig, ax = plt.subplots(figsize=(14, 3.5))

    times = [s["t"] for s in samples]
    rt, rv = zip(*rtf_pts)

    # Background zones: green >= 0.95, amber 0.80-0.95, red < 0.80
    ax.axhspan(0.95, 1.15, alpha=0.06, color="#2e7d32", zorder=0)
    ax.axhspan(0.80, 0.95, alpha=0.07, color="#f9a825", zorder=0)
    ax.axhspan(0.00, 0.80, alpha=0.07, color="#d32f2f", zorder=0)

    _add_exercise_bands(ax, exercise_events, times, 1.15)
    ax.plot(rt, rv, color="#1565c0", linewidth=2, label="RTF", zorder=3)
    ax.fill_between(rt, rv, alpha=0.10, color="#1565c0", zorder=2)
    ax.axhline(
        y=1.0, color="#2e7d32", linewidth=1, linestyle="--", alpha=0.6, label="Real-time (1.0)", zorder=4
    )

    ax.set_ylim(0, 1.15)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Real-Time Factor")
    ax.set_title(
        "Gazebo RTF over time  (gaps = no active simulation)", fontweight="bold", fontsize=13
    )
    ax.legend(fontsize=9, loc="lower right")
    fig.tight_layout()
    return _fig_to_b64(fig)


def chart_process_bars(inventory):
    _chart_setup()
    plt.rcParams.update({"axes.spines.left": False})
    top = [p for p in inventory if p["avg_cpu"] >= 0.3][:12]
    if not top:
        return None

    labels = [p["name"][:48] for p in reversed(top)]
    avgs = [p["avg_cpu"] for p in reversed(top)]
    peaks = [p["peak_cpu"] for p in reversed(top)]
    colors = [CATEGORY_COLOR.get(p["cat"], "#9e9e9e") for p in reversed(top)]

    fig, ax = plt.subplots(figsize=(14, max(4, len(top) * 0.65)))
    y = list(range(len(labels)))
    ax.barh(y, peaks, color=colors, alpha=0.20, height=0.55)
    bars = ax.barh(y, avgs, color=colors, alpha=0.88, height=0.55)
    for bar, v, vp in zip(bars, avgs, peaks):
        ax.text(
            v + 0.5,
            bar.get_y() + bar.get_height() / 2,
            f"  avg {v:.0f}%  peak {vp:.0f}%",
            va="center",
            fontsize=8.5,
            color="#333",
        )
    ax.set_yticks(y)
    ax.set_yticklabels(labels)
    ax.set_xlabel("CPU %")
    ax.set_title(
        "CPU per process  (solid bar = avg, light bar = peak)", fontweight="bold", fontsize=13
    )
    ax.grid(axis="x", alpha=0.35)
    ax.set_axisbelow(True)
    fig.tight_layout()
    return _fig_to_b64(fig)


def chart_memory(samples, exercise_events):
    _chart_setup()
    fig, ax1 = plt.subplots(figsize=(14, 3.5))

    times = [s["t"] for s in samples]
    cont_mem = [s["container"]["mem_mb"] / 1024 for s in samples]
    host_mem = [s["host"]["mem_used_gb"] for s in samples]

    _add_exercise_bands(ax1, exercise_events, times, max(cont_mem) * 1.25)
    ax2 = ax1.twinx()
    (l1,) = ax1.plot(times, cont_mem, color="#e65100", linewidth=2, label="Container RAM (GiB)")
    ax1.fill_between(times, cont_mem, alpha=0.08, color="#e65100")
    (l2,) = ax2.plot(
        times, host_mem, color="#555", linewidth=1, linestyle="--", alpha=0.5, label="Host RAM (GiB)"
    )
    ax1.set_ylim(0, max(cont_mem) * 1.25)
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Container RAM (GiB)", color="#e65100")
    ax2.set_ylabel("Host RAM (GiB)", color="#555")
    ax1.set_title("Memory usage over time", fontweight="bold", fontsize=13)
    ax1.legend([l1, l2], [l1.get_label(), l2.get_label()], fontsize=9)
    fig.tight_layout()
    return _fig_to_b64(fig)


def chart_gpu(samples, exercise_events):
    gpu_samples = [s for s in samples if s.get("gpu")]
    if not gpu_samples:
        return None

    _chart_setup()
    fig, (ax_util, ax_detail) = plt.subplots(1, 2, figsize=(14, 3))

    times = [s["t"] for s in gpu_samples]
    util = [s["gpu"]["util_pct"] for s in gpu_samples]

    _add_exercise_bands(ax_util, exercise_events, times, max(util) * 1.2 or 10)
    ax_util.plot(times, util, color="#6a1b9a", linewidth=2)
    ax_util.fill_between(times, util, alpha=0.1, color="#6a1b9a")
    ax_util.set_ylim(0, max(util) * 1.2 if max(util) > 0 else 10)
    ax_util.set_xlabel("Time (s)")
    ax_util.set_ylabel("GPU %")
    ax_util.set_title("GPU utilization", fontweight="bold")

    mem = [s["gpu"].get("mem_used_mb") or 0 for s in gpu_samples]
    temp = [s["gpu"].get("temp_c") for s in gpu_samples]
    if any(m > 0 for m in mem):
        ax_detail.plot(times, mem, color="#0277bd", linewidth=2, label="VRAM (MB)")
        ax_detail.set_ylabel("VRAM (MB)", color="#0277bd")
    if any(t is not None for t in temp):
        ax_t = ax_detail.twinx()
        ax_t.plot(
            times,
            [t or 0 for t in temp],
            color="#e65100",
            linewidth=1.5,
            linestyle="--",
            label="Temp (°C)",
        )
        ax_t.set_ylabel("Temp (°C)", color="#e65100")
    ax_detail.set_xlabel("Time (s)")
    ax_detail.set_title("VRAM and temperature", fontweight="bold")
    fig.tight_layout()
    return _fig_to_b64(fig)


def chart_category_cpu(samples, exercise_events):
    """Stacked area chart showing how total CPU is split across process categories."""
    times, series = category_cpu_over_time(samples)

    # Only include categories that are meaningfully active
    active = {cat: vals for cat, vals in series.items() if max(vals, default=0) >= 1.0}
    if not active:
        return None

    _chart_setup()
    fig, ax = plt.subplots(figsize=(14, 4.5))

    cats = list(active.keys())
    vals = [active[c] for c in cats]
    colors = [CATEGORY_COLOR.get(c, "#9e9e9e") for c in cats]

    ax.stackplot(times, vals, labels=cats, colors=colors, alpha=0.82)

    # Draw exercise transitions as vertical dashed lines on top of the stacked area
    for i, (t0, name, *_) in enumerate(exercise_events):
        if name != "idle":
            color = _EXC_PALETTE[i % len(_EXC_PALETTE)]
            ax.axvline(t0, color=color, linewidth=1.2, linestyle="--", alpha=0.7, zorder=5)
            ymax = ax.get_ylim()[1] or 10
            ax.text(
                t0 + (times[-1] - times[0]) * 0.005,
                ymax * 0.96,
                name.replace("_harmonic", "").replace("_", " "),
                fontsize=7.5,
                color=color,
                va="top",
                bbox=dict(boxstyle="round,pad=0.15", facecolor="white", edgecolor=color, alpha=0.88),
                zorder=6,
            )

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("CPU %  (sum of per-process measurements)")
    ax.set_title("CPU distribution by category over time", fontweight="bold", fontsize=13)
    ax.legend(loc="upper left", fontsize=9, ncol=3, framealpha=0.92)
    fig.tight_layout()
    return _fig_to_b64(fig)


def chart_network(samples):
    """Line chart of container network throughput (MB/s) derived from docker stats deltas."""
    if len(samples) < 2:
        return None

    rx_cum = [s["container"].get("net_rx_mb", 0) for s in samples]
    tx_cum = [s["container"].get("net_tx_mb", 0) for s in samples]
    times = [s["t"] for s in samples]

    # Session totals (cumulative values from container start, so we take the delta)
    total_rx = max(0.0, rx_cum[-1] - rx_cum[0])
    total_tx = max(0.0, tx_cum[-1] - tx_cum[0])
    if total_rx < 0.5 and total_tx < 0.5:
        return None  # No meaningful network activity

    # Per-interval throughput in MB/s
    dt = [max(times[i] - times[i - 1], 0.1) for i in range(1, len(times))]
    rx_bps = [max(0.0, rx_cum[i] - rx_cum[i - 1]) / dt[i - 1] for i in range(1, len(rx_cum))]
    tx_bps = [max(0.0, tx_cum[i] - tx_cum[i - 1]) / dt[i - 1] for i in range(1, len(tx_cum))]
    t_mid = times[1:]

    _chart_setup()
    fig, ax = plt.subplots(figsize=(14, 3))
    ax.plot(t_mid, rx_bps, color="#1565c0", linewidth=1.8, label=f"RX  (total {total_rx:.1f} MB)")
    ax.plot(
        t_mid,
        tx_bps,
        color="#e65100",
        linewidth=1.8,
        linestyle="--",
        label=f"TX  (total {total_tx:.1f} MB)",
    )
    ax.fill_between(t_mid, rx_bps, alpha=0.10, color="#1565c0")
    ax.fill_between(t_mid, tx_bps, alpha=0.10, color="#e65100")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("MB/s")
    ax.set_title("Container network throughput", fontweight="bold", fontsize=13)
    ax.legend(fontsize=9)
    fig.tight_layout()
    return _fig_to_b64(fig)
