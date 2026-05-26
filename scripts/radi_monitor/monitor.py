"""
monitor.py — Entry point for radi_monitor.

This is the only file you need to run. Helper modules live in lib/:
    lib/collector.py  — Docker/container/GPU/host data collection
    lib/analysis.py   — Statistics, exercise events, measurement summary
    lib/charts.py     — Matplotlib chart generation
    lib/report.py     — HTML template, CSS, PDF export

Reports are written to /tmp/ by default (override with --output).

How it works:
    Attaches to a running jderobot/robotics-backend Docker container and samples
    at a configurable interval: container CPU/RAM (docker stats), per-process CPU/RAM
    (psutil inside the container), Gazebo RTF (/stats topic), GPU utilization
    (nvidia-smi or /sys/class/drm), and host CPU/RAM. All five collectors run in
    parallel to minimise overhead.

    On Ctrl+C the session is flushed to JSON and a self-contained HTML report
    (with embedded charts) is generated. A PDF is produced automatically if
    weasyprint is installed.

Exercise detection:
    Automatic — the active .world filename is parsed from the gz sim / gzserver
    cmdline. Manual tagging is also supported: type a name and press Enter at any
    time to label the current period; an empty Enter clears the tag.

Usage:
    python3 monitor.py
    python3 monitor.py --interval 5
    python3 monitor.py --output /tmp/my_session
    python3 monitor.py --container <name_or_id>
    python3 monitor.py --verbose

Dependencies:
    pip3 install psutil matplotlib weasyprint
"""

import argparse
import queue
import signal
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime

import psutil

from lib.analysis import _dur, get_exercise_events
from lib.collector import (
    check_container_gpu,
    collect_system_info,
    container_processes,
    detect_exercise,
    docker_stats,
    find_container,
    gazebo_rtf,
    gpu_stats,
    host_stats,
    save_session,
)
from lib.report import PDF_AVAILABLE, generate_report

VERBOSE = False


def log(*args, **kwargs):
    if VERBOSE:
        print(*args, **kwargs)


def main():
    parser = argparse.ArgumentParser(
        description="Performance monitor for jderobot/robotics-backend"
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=3.0,
        help="Sampling interval in seconds (default: 3)",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Output base path without extension (default: /tmp/radi_TIMESTAMP)",
    )
    parser.add_argument(
        "--container",
        default=None,
        help="Container name or ID (auto-detected if omitted)",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print internal progress messages",
    )
    args = parser.parse_args()

    global VERBOSE
    VERBOSE = args.verbose

    ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = args.output or f"/tmp/radi_{ts_str}"

    print(f"Output: {base}.html / .pdf / .json")
    log("Collecting system info...", end=" ", flush=True)
    sysinfo = collect_system_info()
    log(sysinfo.get("cpu", "?")[:50])

    input_queue: queue.Queue = queue.Queue()

    def _stdin_reader():
        while True:
            try:
                line = sys.stdin.readline()
                if not line:
                    break
                input_queue.put(line.rstrip("\n"))
            except Exception:
                break

    threading.Thread(target=_stdin_reader, daemon=True).start()
    psutil.cpu_percent(interval=None)

    samples = []
    manual_events = []
    manual_tag = None
    container_id = None
    container_name = None
    container_img = None
    session_start = None
    last_exercise = None
    running = True
    header_printed = False
    meta = {}

    def on_stop(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, on_stop)
    signal.signal(signal.SIGTERM, on_stop)

    pool = ThreadPoolExecutor(max_workers=5)
    try:
        while running:
            try:
                while True:
                    typed = input_queue.get_nowait().strip()
                    t_now = time.time() - (meta.get("_start_time") or time.time())
                    if typed:
                        manual_tag = typed
                        manual_events.append({"t": round(t_now, 2), "name": manual_tag})
                        print(f"[tag] {manual_tag} at t={t_now:.1f}s")
                    else:
                        manual_tag = None
                        print("[tag] cleared")
            except queue.Empty:
                pass

            if container_id is None:
                container_id, container_name, container_img = find_container(
                    args.container
                )
                if container_id is None:
                    print("Waiting for container...", end="\r")
                    time.sleep(2)
                    continue
                start_time = time.time()
                session_start = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                container_gpu = check_container_gpu(container_id)
                meta = {
                    "container": container_name,
                    "image": container_img or "jderobot/robotics-backend:latest",
                    "interval_s": args.interval,
                    "manual_events": manual_events,
                    "session_start": session_start,
                    "sysinfo": sysinfo,
                    "container_gpu": container_gpu,
                    "_start_time": start_time,
                }
                log(
                    f"Container GPU: {'yes — ' + container_gpu if container_gpu else 'not accessible'}"
                )
                print(
                    f"Monitoring {container_name} ({container_id[:12]}) · {container_img}"
                )

            t0 = time.time()
            elapsed = t0 - meta["_start_time"]
            ts = datetime.now().isoformat()

            # Run all five collectors in parallel — reduces effective overhead from ~2.5s to ~1.2s
            f_cstats = pool.submit(docker_stats, container_id)
            f_procs = pool.submit(container_processes, container_id)
            f_rtf = pool.submit(gazebo_rtf, container_id)
            f_gpu = pool.submit(gpu_stats)
            f_hst = pool.submit(host_stats)
            cstats = f_cstats.result()
            processes = f_procs.result()
            rtf = f_rtf.result()
            gpu = f_gpu.result()
            hst = f_hst.result()

            if cstats is None:
                print("Container stopped — waiting...")
                container_id = None
                time.sleep(3)
                continue

            auto_exc = detect_exercise(processes)
            exercise = manual_tag if manual_tag else auto_exc

            sample = {
                "t": round(elapsed, 2),
                "ts": ts,
                "container": cstats,
                "processes": processes[:40],
                "exercise": exercise,
                "exercise_source": "manual"
                if manual_tag
                else ("auto" if auto_exc else None),
                "rtf": rtf,
                "gpu": gpu,
                "host": hst,
            }
            samples.append(sample)

            if not header_printed:
                print(
                    f"{'Time':>8}  {'CPU':>9}  {'RAM':>8}  {'GPU':>5}  {'RTF':>6}  Exercise"
                )
                header_printed = True

            gpu_s = f"{gpu['util_pct']:4.0f}%" if gpu else "  N/A"
            rtf_s = f"{rtf:.3f}" if rtf is not None else "   N/A"
            exc_s = (exercise or "—")[:26]
            new = " ←" if exercise and exercise != last_exercise else ""
            tag_m = " [M]" if manual_tag else ""
            print(
                f"{elapsed:7.1f}s  {cstats['cpu_pct']:8.1f}%  {cstats['mem_mb']:6.0f}MB"
                f"  {gpu_s}  {rtf_s}  {exc_s}{tag_m}{new}"
            )
            last_exercise = exercise

            save_session(base + ".json", samples, meta)

            overhead = time.time() - t0
            time.sleep(max(0.1, args.interval - overhead))
    finally:
        pool.shutdown(wait=False)

    if not samples:
        print("No samples collected.")
        return

    session_end = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    meta["session_start"] = meta.get("session_start") or session_end
    meta.pop("_start_time", None)
    save_session(base + ".json", samples, meta)

    print("\nGenerating report...")
    log(f"  {len(samples)} samples over {_dur(samples[-1]['t'])}")

    html_path, pdf_path, summary = generate_report(samples, meta, base, verbose=VERBOSE)

    print(f"Report: {html_path}")
    if pdf_path:
        print(f"PDF:    {pdf_path}")
    elif not PDF_AVAILABLE:
        print("PDF:    not generated (pip3 install weasyprint --break-system-packages)")

    dur = samples[-1]["t"]
    excs = [e[1] for e in get_exercise_events(samples, manual_events) if e[1] != "idle"]
    print(
        f"\nSession    {session_start} → {session_end}  ({_dur(dur)}, {len(samples)} samples)"
    )
    print(
        f"CPU        avg {summary['cpu_avg']}%  peak {summary['cpu_peak']}%"
        f"  (of {summary['cpu_max_pct']}% max, {meta['sysinfo'].get('cpu_threads','?')} threads)"
    )
    print(
        f"RAM        avg {summary['mem_avg_gb']} GiB  peak {summary['mem_peak_gb']} GiB"
    )
    if summary.get("rtf_avg") is not None:
        print(f"RTF        avg {summary['rtf_avg']}  min {summary['rtf_min']}")
    if summary.get("gpu_avg") is not None:
        print(f"GPU        avg {summary['gpu_avg']}%  peak {summary['gpu_peak']}%")
    if excs:
        print(f"Exercises  {', '.join(excs)}")


if __name__ == "__main__":
    main()
