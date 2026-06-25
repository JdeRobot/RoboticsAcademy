"""
report.py — HTML report generation and PDF export for radi_monitor.

Builds a self-contained HTML file (all charts embedded as base64 PNG) and
optionally converts it to PDF using weasyprint.
"""

from datetime import datetime

try:
    from weasyprint import HTML as WeasyHTML

    PDF_AVAILABLE = True
except ImportError:
    PDF_AVAILABLE = False

from .analysis import (
    _dur,
    get_exercise_events,
    measurement_summary,
    per_exercise_stats,
    process_inventory,
    session_summary,
)
from .charts import (
    chart_category_cpu,
    chart_cpu,
    chart_gpu,
    chart_memory,
    chart_network,
    chart_process_bars,
    chart_rtf,
)
from .collector import CATEGORY_COLOR

_CSS = """
body{font-family:Arial,sans-serif;font-size:14px;line-height:1.6;
     max-width:1300px;margin:0 auto;padding:24px;color:#222;background:#fff}
h1{font-size:21px;border-bottom:3px solid #d32f2f;padding-bottom:6px;margin-bottom:4px}
h2{font-size:14px;margin:28px 0 10px;border-left:4px solid #1565c0;
   padding-left:10px;color:#1565c0;text-transform:uppercase;letter-spacing:.5px}
.meta{color:#777;font-size:12px;margin-bottom:6px}
.sysinfo{background:#f5f5f5;border-radius:4px;padding:8px 14px;font-size:12px;
         color:#444;margin-bottom:20px;line-height:1.9}
.kpi{display:flex;gap:10px;flex-wrap:wrap;margin-bottom:20px}
.kb{border:1px solid #ddd;border-radius:6px;padding:11px 16px;min-width:125px;flex:1}
.kl{font-size:10px;color:#999;text-transform:uppercase;font-weight:bold}
.kv{font-size:26px;font-weight:bold;color:#d32f2f;line-height:1.1}
.ks{font-size:11px;color:#888}
.diag{margin-bottom:20px}
.d{display:flex;gap:10px;padding:8px 14px;border-radius:4px;margin:5px 0;border-left:5px solid}
.d.ok{background:#f1f8e9;border-color:#558b2f}
.d.warn{background:#fff8e1;border-color:#f9a825}
.d.crit{background:#ffebee;border-color:#d32f2f}
.dm{font-size:13px;font-weight:bold}
.df{font-size:12px;color:#555;font-style:italic}
table{width:100%;border-collapse:collapse;margin:8px 0 22px;font-size:13px}
th{background:#f0f0f0;border:1px solid #ddd;padding:8px 12px;
   text-align:left;font-size:11px;text-transform:uppercase;color:#555}
td{border:1px solid #eee;padding:7px 12px;vertical-align:middle}
tr:nth-child(even) td{background:#fafafa}
.r{text-align:right;font-variant-numeric:tabular-nums}
.hi{color:#d32f2f;font-weight:bold}
.mid{color:#e65100;font-weight:bold}
.okv{color:#2e7d32}
.tagm{background:#ede7f6;color:#4a148c;padding:2px 7px;border-radius:3px;
      font-size:11px;font-weight:bold}
.dot{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:5px}
.chart{margin:8px 0 26px}
.chart img{max-width:100%;border:1px solid #eee;border-radius:4px;display:block}
.rtf-ok{color:#2e7d32;font-weight:bold}
.rtf-warn{color:#e65100;font-weight:bold}
.rtf-bad{color:#d32f2f;font-weight:bold}
footer{color:#bbb;font-size:11px;text-align:center;margin-top:40px;
       padding-top:12px;border-top:1px solid #eee}
@media print{.chart img{page-break-inside:avoid}h2{page-break-after:avoid}}
"""


def _cpu_cls(v):
    return "hi" if v > 200 else "mid" if v > 80 else "okv"


def _rtf_cell(v):
    if v is None:
        return "—"
    cls = "rtf-ok" if v >= 0.95 else "rtf-warn" if v >= 0.80 else "rtf-bad"
    return f'<span class="{cls}">{v:.3f}</span>'


def generate_report(samples, meta, output_base, verbose=False):
    def _log(*args, **kwargs):
        if verbose:
            print(*args, **kwargs)

    sysinfo = meta.get("sysinfo", {})
    manual_events = meta.get("manual_events", [])
    session_start = meta.get("session_start", "")
    cpu_threads = sysinfo.get("cpu_threads", 1)

    exercise_events = get_exercise_events(samples, manual_events)
    summary = session_summary(samples, cpu_threads)
    exc_stats = per_exercise_stats(samples, exercise_events)
    metrics_rows = measurement_summary(samples, summary)
    inventory = process_inventory(samples)

    _log("  rendering CPU chart...", end=" ", flush=True)
    img_cpu = chart_cpu(samples, exercise_events, cpu_threads)
    _log("done")
    _log("  rendering RTF chart...", end=" ", flush=True)
    img_rtf = chart_rtf(samples, exercise_events)
    _log("done" if img_rtf else "no RTF data")
    _log("  rendering category CPU chart...", end=" ", flush=True)
    img_cat = chart_category_cpu(samples, exercise_events)
    _log("done")
    _log("  rendering process bars...", end=" ", flush=True)
    img_proc = chart_process_bars(inventory)
    _log("done")
    _log("  rendering memory chart...", end=" ", flush=True)
    img_mem = chart_memory(samples, exercise_events)
    _log("done")
    _log("  rendering GPU chart...", end=" ", flush=True)
    img_gpu = chart_gpu(samples, exercise_events)
    _log("done" if img_gpu else "no GPU data")
    _log("  rendering network chart...", end=" ", flush=True)
    img_net = chart_network(samples)
    _log("done" if img_net else "skipped (no significant traffic)")

    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    has_gpu = summary.get("gpu_avg") is not None

    # System info block — includes nvidia-smi output if available
    gpu_str = " · ".join(sysinfo.get("gpus", ["not detected"]))
    cgpu = meta.get("container_gpu")
    cgpu_str = (
        f"yes — {cgpu}"
        if cgpu
        else "not accessible (container started without --gpus all)"
    )
    nvidia_smi_html = ""
    if sysinfo.get("nvidia_smi"):
        nvidia_smi_html = (
            f'<details style="margin-top:6px">'
            f'<summary style="cursor:pointer;font-size:11px;color:#1565c0">nvidia-smi output</summary>'
            f'<pre style="font-size:10px;background:#f8f8f8;padding:8px;border-radius:3px;'
            f'overflow-x:auto;margin-top:4px">{sysinfo["nvidia_smi"]}</pre></details>'
        )
    sysblock = (
        f'<div class="sysinfo">'
        f'<strong>CPU:</strong> {sysinfo.get("cpu", "?")} '
        f'({sysinfo.get("cpu_cores", "?")} cores / {sysinfo.get("cpu_threads", "?")} threads)'
        f" &nbsp;·&nbsp; <strong>RAM:</strong> {sysinfo.get('ram_gb', '?')} GiB<br>"
        f"<strong>Host GPU:</strong> {gpu_str}<br>"
        f"<strong>Container GPU access:</strong> {cgpu_str}<br>"
        f'<strong>OS:</strong> {sysinfo.get("os", "?")} (kernel {sysinfo.get("kernel", "?")})'
        f" &nbsp;·&nbsp; "
        f'<strong>Image:</strong> {meta.get("image", "jderobot/robotics-backend:latest")}'
        f' &nbsp;·&nbsp; <strong>Container:</strong> {meta.get("container", "?")}'
        f"{nvidia_smi_html}"
        f"</div>"
    )

    rtf_val = (
        f"{summary['rtf_avg']:.3f}" if summary.get("rtf_avg") is not None else "N/A"
    )
    rtf_sub = (
        f"min {summary['rtf_min']:.3f}"
        if summary.get("rtf_min") is not None
        else "no simulation detected"
    )
    kpi = (
        f'<div class="kpi">'
        f'<div class="kb"><div class="kl">Container CPU</div>'
        f'<div class="kv">{summary["cpu_avg"]}%</div>'
        f'<div class="ks">avg · peak {summary["cpu_peak"]}% · max {summary["cpu_max_pct"]}%</div></div>'
        f'<div class="kb"><div class="kl">Container RAM</div>'
        f'<div class="kv">{summary["mem_avg_gb"]} GiB</div>'
        f'<div class="ks">avg · peak {summary["mem_peak_gb"]} GiB</div></div>'
        f'<div class="kb"><div class="kl">Gazebo RTF</div>'
        f'<div class="kv">{rtf_val}</div>'
        f'<div class="ks">{rtf_sub}</div></div>'
        f'<div class="kb"><div class="kl">GPU</div>'
        f'<div class="kv">{"{}%".format(summary["gpu_avg"]) if has_gpu else "N/A"}</div>'
        f'<div class="ks">{"peak {}%".format(summary["gpu_peak"]) if has_gpu else "not detected"}'
        f"</div></div>"
        f'<div class="kb"><div class="kl">Host CPU</div>'
        f'<div class="kv">{summary["host_cpu_avg"]}%</div>'
        f'<div class="ks">avg during session</div></div>'
        f'<div class="kb"><div class="kl">Duration</div>'
        f'<div class="kv">{_dur(summary["duration"])}</div>'
        f'<div class="ks">{summary["n"]} samples · every {meta.get("interval_s", "?")}s</div></div>'
        f"</div>"
    )

    metrics_rows_html = "".join(
        f'<tr><td>{m}</td><td class="r">{avg}</td><td class="r">{peak}</td>'
        f'<td style="color:#666;font-size:12px">{detail}</td></tr>\n'
        for m, avg, peak, detail in metrics_rows
    )
    metrics_table = (
        '<table><tr><th>Metric</th><th class="r">Average</th>'
        '<th class="r">Peak / Min</th><th>Details</th></tr>'
        + metrics_rows_html
        + "</table>"
    )

    if exc_stats:
        rows = ""
        for e in exc_stats:
            tag = (
                ' <span class="tagm">[manual]</span>' if e["source"] == "manual" else ""
            )
            name = e["name"].replace("_harmonic", "").replace("_", " ")
            rows += (
                f"<tr><td><strong>{name}</strong>{tag}</td>"
                f'<td class="r">{_dur(e["duration"])}</td>'
                f'<td class="r {_cpu_cls(e["cpu_avg"])}">{e["cpu_avg"]}%</td>'
                f'<td class="r {_cpu_cls(e["cpu_peak"])}">{e["cpu_peak"]}%</td>'
                f'<td class="r">{e["ram_avg"]} GiB</td>'
                f'<td class="r">{"{}%".format(e["gpu_avg"]) if e["gpu_avg"] is not None else "—"}'
                f"</td>"
                f'<td class="r">{_rtf_cell(e["rtf_avg"])}</td>'
                f'<td class="r">{_rtf_cell(e["rtf_min"])}</td></tr>\n'
            )
        exc_table = (
            '<table><tr><th>Exercise</th><th class="r">Duration</th>'
            '<th class="r">CPU avg</th><th class="r">CPU peak</th>'
            '<th class="r">RAM avg</th><th class="r">GPU avg</th>'
            '<th class="r">RTF avg</th><th class="r">RTF min</th></tr>'
            + rows
            + "</table>"
        )
    else:
        exc_table = (
            '<p style="color:#999;padding:8px 0">No exercises detected. '
            "Use manual tagging: type a name and press Enter while monitoring.</p>"
        )

    proc_rows = ""
    for p in inventory:
        color = CATEGORY_COLOR.get(p["cat"], "#9e9e9e")
        proc_rows += (
            f'<tr><td><code>{p["name"]}</code></td>'
            f'<td><span class="dot" style="background:{color}"></span>'
            f'<span style="color:{color};font-size:11px">{p["cat"]}</span></td>'
            f'<td class="r {_cpu_cls(p["avg_cpu"])}">{p["avg_cpu"]}%</td>'
            f'<td class="r {_cpu_cls(p["peak_cpu"])}">{p["peak_cpu"]}%</td>'
            f'<td class="r">{p["avg_mem"]:.0f} MB</td>'
            f'<td class="r">{p["pct_time"]}%</td></tr>\n'
        )
    proc_table = (
        "<table><tr><th>Process</th><th>Category</th>"
        '<th class="r">CPU avg</th><th class="r">CPU peak</th>'
        '<th class="r">RAM avg</th><th class="r">% time active</th></tr>'
        + proc_rows
        + "</table>"
    )

    rtf_section = (
        f"<h2>Gazebo RTF</h2>"
        f'<div class="chart"><img src="data:image/png;base64,{img_rtf}"></div>'
        if img_rtf
        else ""
    )
    cat_section = (
        f"<h2>CPU by category</h2>"
        f'<div class="chart"><img src="data:image/png;base64,{img_cat}"></div>'
        if img_cat
        else ""
    )
    gpu_section = (
        f"<h2>GPU</h2>"
        f'<div class="chart"><img src="data:image/png;base64,{img_gpu}"></div>'
        if img_gpu
        else ""
    )
    net_section = (
        f"<h2>Network I/O</h2>"
        f'<div class="chart"><img src="data:image/png;base64,{img_net}"></div>'
        if img_net
        else ""
    )

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RADI Performance Report — {session_start}</title>
<style>{_CSS}</style>
</head>
<body>
<h1>RADI Performance Report</h1>
<div class="meta">
Session: <strong>{session_start}</strong> → <strong>{now}</strong>
&nbsp;·&nbsp; Report generated: {now}
&nbsp;·&nbsp; <em>PDF: Ctrl+P → Save as PDF in your browser</em>
</div>
{sysblock}
{kpi}
<h2>Session metrics</h2>
{metrics_table}
<h2>Resource usage per exercise</h2>
{exc_table}
<h2>CPU</h2>
<div class="chart"><img src="data:image/png;base64,{img_cpu}"></div>
{rtf_section}
{cat_section}
<h2>CPU per process</h2>
<div class="chart"><img src="data:image/png;base64,{img_proc}"></div>
{gpu_section}
<h2>Memory</h2>
<div class="chart"><img src="data:image/png;base64,{img_mem}"></div>
{net_section}
<h2>Process inventory</h2>
{proc_table}
<footer>jderobot/robotics-backend · radi_monitor · {now}</footer>
</body>
</html>"""

    html_path = output_base + ".html"
    pdf_path = output_base + ".pdf"

    with open(html_path, "w", encoding="utf-8") as f:
        f.write(html)

    if PDF_AVAILABLE:
        _log("  generating PDF...", end=" ", flush=True)
        try:
            WeasyHTML(filename=html_path).write_pdf(pdf_path)
            _log("done")
        except Exception as e:
            _log(f"failed: {e}")
            pdf_path = None
    else:
        pdf_path = None

    return html_path, pdf_path, summary
