"""
analysis.py — Statistics computation and diagnosis for radi_monitor.

Derives per-session and per-exercise summaries, process inventory, exercise event
timeline, and an automatic diagnosis with actionable findings.
"""

from collections import defaultdict

from .collector import CATEGORY_COLOR, categorize, detect_exercise, friendly_name


def _dur(sec):
    m, s = divmod(int(sec), 60)
    return f"{m}m {s:02d}s" if m else f"{s}s"


def get_exercise_events(samples, manual_events):
    # Build events without mutating the sample dicts
    events, prev = [], None
    for s in samples:
        if s.get("exercise_source") != "manual":
            exc = detect_exercise(s.get("processes", []))
        else:
            exc = s.get("exercise")
        label = exc or "idle"
        if label != prev:
            events.append((s["t"], label, "auto"))
            prev = label

    for ev in manual_events or []:
        events.append((ev["t"], ev["name"], "manual"))
    events.sort(key=lambda x: x[0])

    deduped, prev_name = [], None
    for t, name, src in events:
        if name != prev_name:
            deduped.append((t, name, src))
            prev_name = name
    return deduped


def session_summary(samples, cpu_threads):
    cpus = [s["container"]["cpu_pct"] for s in samples]
    mems = [s["container"]["mem_mb"] for s in samples]
    hcpu = [s["host"]["cpu_pct"] for s in samples]
    gpus = [s["gpu"]["util_pct"] for s in samples if s.get("gpu")]
    rtfs = [s["rtf"] for s in samples if s.get("rtf") is not None]
    return {
        "duration": samples[-1]["t"],
        "n": len(samples),
        "cpu_avg": round(sum(cpus) / len(cpus), 1),
        "cpu_peak": round(max(cpus), 1),
        "cpu_max_pct": cpu_threads * 100,
        "mem_avg_gb": round(sum(mems) / len(mems) / 1024, 2),
        "mem_peak_gb": round(max(mems) / 1024, 2),
        "host_cpu_avg": round(sum(hcpu) / len(hcpu), 1),
        "gpu_avg": round(sum(gpus) / len(gpus), 1) if gpus else None,
        "gpu_peak": round(max(gpus), 1) if gpus else None,
        "rtf_avg": round(sum(rtfs) / len(rtfs), 3) if rtfs else None,
        "rtf_min": round(min(rtfs), 3) if rtfs else None,
    }


def per_exercise_stats(samples, exercise_events):
    rows = []
    for i, (t0, name, src) in enumerate(exercise_events):
        t1 = (
            exercise_events[i + 1][0]
            if i + 1 < len(exercise_events)
            else samples[-1]["t"]
        )
        w = [s for s in samples if t0 <= s["t"] <= t1]
        if not w:
            continue
        cpus = [s["container"]["cpu_pct"] for s in w]
        mems = [s["container"]["mem_mb"] for s in w]
        gpus = [s["gpu"]["util_pct"] for s in w if s.get("gpu")]
        rtfs = [s["rtf"] for s in w if s.get("rtf") is not None]
        rows.append(
            {
                "name": name,
                "source": src,
                "duration": t1 - t0,
                "cpu_avg": round(sum(cpus) / len(cpus), 1),
                "cpu_peak": round(max(cpus), 1),
                "ram_avg": round(sum(mems) / len(mems) / 1024, 2),
                "gpu_avg": round(sum(gpus) / len(gpus), 1) if gpus else None,
                "rtf_avg": round(sum(rtfs) / len(rtfs), 3) if rtfs else None,
                "rtf_min": round(min(rtfs), 3) if rtfs else None,
            }
        )
    rows.sort(key=lambda x: -x["cpu_avg"])
    return rows


def process_inventory(samples):
    data = defaultdict(lambda: {"cpu": [], "mem": [], "seen": 0, "cmd": ""})
    for s in samples:
        seen = set()
        for p in s.get("processes", []):
            key = friendly_name(p)
            if key in seen:
                continue
            seen.add(key)
            data[key]["cpu"].append(p.get("cpu", 0))
            data[key]["mem"].append(p.get("mem_mb", 0))
            data[key]["seen"] += 1
            data[key]["cmd"] = p.get("cmd", "")
    total = len(samples)
    out = []
    for name, d in data.items():
        n = len(d["cpu"])
        out.append(
            {
                "name": name,
                "cat": categorize({"name": name, "cmd": d["cmd"]}),
                "avg_cpu": round(sum(d["cpu"]) / n, 1),
                "peak_cpu": round(max(d["cpu"]), 1),
                "avg_mem": round(sum(d["mem"]) / n, 1),
                "pct_time": round(d["seen"] / total * 100),
            }
        )
    out.sort(key=lambda x: -x["avg_cpu"])
    return out[:15]


def measurement_summary(samples, summary):
    """Return objective (metric, avg, peak, detail) rows, with no interpretation."""
    rows = []
    max_pct = summary["cpu_max_pct"]
    threads = max_pct // 100

    rows.append(
        (
            "Container CPU",
            f"{summary['cpu_avg']}%",
            f"{summary['cpu_peak']}%",
            f"max {max_pct}%  ({threads} logical cores)",
        )
    )

    mems = [s["container"]["mem_mb"] for s in samples]
    mem_change = (sum(mems[-5:]) / 5 - sum(mems[:5]) / 5) if len(mems) >= 10 else None
    mem_detail = f"net change {mem_change:+.0f} MB" if mem_change is not None else "—"
    rows.append(
        (
            "Container RAM",
            f"{summary['mem_avg_gb']} GiB",
            f"{summary['mem_peak_gb']} GiB",
            mem_detail,
        )
    )

    rtf_samples = [s for s in samples if s.get("rtf") is not None]
    if summary.get("rtf_avg") is not None:
        rows.append(
            (
                "Gazebo RTF",
                f"{summary['rtf_avg']:.3f}",
                f"min {summary['rtf_min']:.3f}",
                f"{len(rtf_samples)} / {len(samples)} samples",
            )
        )
    else:
        rows.append(("Gazebo RTF", "—", "—", "no samples with active simulation"))

    if summary.get("gpu_avg") is not None:
        rows.append(
            (
                "GPU utilization",
                f"{summary['gpu_avg']}%",
                f"{summary['gpu_peak']}%",
                "—",
            )
        )
    else:
        rows.append(("GPU utilization", "—", "—", "not detected on host"))

    rows.append(("Host CPU", f"{summary['host_cpu_avg']}%", "—", "—"))

    rx0 = samples[0]["container"].get("net_rx_mb", 0)
    tx0 = samples[0]["container"].get("net_tx_mb", 0)
    rx1 = samples[-1]["container"].get("net_rx_mb", 0)
    tx1 = samples[-1]["container"].get("net_tx_mb", 0)
    rows.append(
        (
            "Network I/O",
            "—",
            "—",
            f"RX {max(0, rx1 - rx0):.1f} MB  ·  TX {max(0, tx1 - tx0):.1f} MB",
        )
    )

    return rows


def category_cpu_over_time(samples):
    """Aggregate per-process CPU into category buckets for each sample.

    Returns (times, series) where series is {category: [float, ...]} aligned to times.
    """
    categories = list(CATEGORY_COLOR.keys())
    times = [s["t"] for s in samples]
    series = {cat: [] for cat in categories}
    for s in samples:
        totals = {cat: 0.0 for cat in categories}
        for p in s.get("processes", []):
            totals[categorize(p)] += p.get("cpu", 0.0)
        for cat in categories:
            series[cat].append(totals[cat])
    return times, series
