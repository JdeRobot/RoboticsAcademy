"""
collector.py — Docker container and host data collection for radi_monitor.

Provides functions to query container CPU/RAM (docker stats), per-process metrics
(psutil inside the container via docker exec), Gazebo RTF (/stats topic), GPU stats
(nvidia-smi or /sys/class/drm), host CPU/RAM, and exercise auto-detection from
process cmdlines.
"""

import json
import os
import platform
import re
import subprocess
import sys

try:
    import psutil
except ImportError:
    sys.exit("psutil not found. Run: pip3 install psutil")

# Image-name substrings that identify a RADI container: robotics-backend (production)
# and robotics-academy (the development image bundling frontend + backend).
RADI_IMAGES = ("robotics-backend", "robotics-academy")

CATEGORY_COLOR = {
    "Gazebo Server": "#d32f2f",
    "Gazebo Client": "#f57c00",
    "RAM Manager": "#1565c0",
    "Django": "#2e7d32",
    "ROS2 Bridges": "#f9a825",
    "ROS2 Nodes": "#e65100",
    "VNC": "#6a1b9a",
    "User code": "#37474f",
    "Other": "#9e9e9e",
}

# Runs inside the container via docker exec; two-pass CPU measurement is required
# because psutil.cpu_percent(interval=None) needs a prior call to initialise counters.
_PSUTIL_SCRIPT = """
import psutil, json, time
procs = {}
for p in psutil.process_iter(['pid', 'name', 'cmdline']):
    try:
        procs[p.pid] = p
        p.cpu_percent(interval=None)
    except Exception:
        pass
time.sleep(0.5)
out = []
for pid, p in procs.items():
    try:
        out.append({'pid': pid, 'name': p.name(),
                    'cpu': round(p.cpu_percent(interval=None), 2),
                    'mem_mb': round(p.memory_info().rss / 1048576, 2),
                    'cmd': ' '.join(p.cmdline()[:6]) if p.cmdline() else p.name()})
    except Exception:
        pass
out.sort(key=lambda x: -x['cpu'])
print(json.dumps(out))
"""


def collect_system_info():
    info = {}
    try:
        out = subprocess.check_output(["lsb_release", "-d"], text=True, timeout=3)
        info["os"] = out.split(":", 1)[1].strip()
    except Exception:
        info["os"] = platform.platform()
    info["kernel"] = platform.release()
    try:
        with open("/proc/cpuinfo") as f:
            for line in f:
                if "model name" in line:
                    info["cpu"] = line.split(":", 1)[1].strip()
                    break
    except Exception:
        info["cpu"] = platform.processor() or "unknown"
    info["cpu_cores"] = psutil.cpu_count(logical=False) or 1
    info["cpu_threads"] = psutil.cpu_count(logical=True) or 1
    info["ram_gb"] = round(psutil.virtual_memory().total / 1024**3, 1)
    gpus = []
    try:
        out = subprocess.check_output(["lspci"], text=True, timeout=5)
        for line in out.splitlines():
            if re.search(r"VGA|3D controller", line, re.I):
                gpus.append(line.split(":", 2)[-1].strip())
    except Exception:
        pass
    try:
        out = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=name,memory.total", "--format=csv,noheader"],
            text=True,
            stderr=subprocess.DEVNULL,
            timeout=3,
        )
        nvidia = [line.strip() for line in out.strip().splitlines() if line.strip()]
        if nvidia:
            gpus = nvidia
    except Exception:
        pass
    info["gpus"] = gpus or ["not detected"]

    # Full nvidia-smi table for the report (None if no NVIDIA GPU)
    try:
        info["nvidia_smi"] = subprocess.check_output(
            ["nvidia-smi"], text=True, stderr=subprocess.DEVNULL, timeout=5
        ).strip()
    except Exception:
        info["nvidia_smi"] = None

    return info


def check_container_gpu(cid):
    """Run nvidia-smi inside the container. Returns a summary string or None."""
    try:
        out = subprocess.check_output(
            [
                "docker",
                "exec",
                cid,
                "nvidia-smi",
                "--query-gpu=name,memory.total,driver_version",
                "--format=csv,noheader",
            ],
            text=True,
            stderr=subprocess.DEVNULL,
            timeout=5,
        )
        if out.strip():
            return out.strip()
    except Exception:
        pass
    return None


def check_container_python(cid):
    """Verify python3 + psutil are usable inside the container.

    Per-process metrics, the CPU-by-category chart and automatic exercise
    detection all depend on this; without it they would stay empty silently.
    """
    try:
        r = subprocess.run(
            ["docker", "exec", cid, "python3", "-c", "import psutil"],
            capture_output=True,
            text=True,
            timeout=8,
        )
        return r.returncode == 0
    except Exception:
        return False


def categorize(p):
    name, cmd = p.get("name", ""), p.get("cmd", "")
    # Gazebo Harmonic runs as 'ruby' with 'gz sim' in the cmdline
    if name == "ruby" or "gz sim" in cmd:
        return "Gazebo Client" if ("-g" in cmd or "--gui" in cmd) else "Gazebo Server"
    c = (name + " " + cmd).lower()
    if "gzserver" in c:
        return "Gazebo Server"
    if "gzclient" in c:
        return "Gazebo Client"
    if "manager.py" in c or "ram_entrypoint" in c:
        return "RAM Manager"
    if "manage.py" in c or "runserver" in c:
        return "Django"
    if "bridge" in c or "gz-transport" in c:
        return "ROS2 Bridges"
    if "/opt/ros" in c or "ros2" in c or "_node" in c or "as2_" in c:
        return "ROS2 Nodes"
    if "tvnc" in c or "xvnc" in c or "novnc" in c or "websockify" in c:
        return "VNC"
    if "python" in c:
        return "User code"
    return "Other"


def friendly_name(p):
    name, cmd = p.get("name", ""), p.get("cmd", "")
    if name == "ruby" or "gz sim" in cmd:
        return (
            "gz sim (client/GUI)" if ("-g" in cmd or "--gui" in cmd) else "gz sim (server/physics)"
        )
    if "ram_entrypoint" in cmd or "manager.py" in cmd:
        return "RAM Manager"
    if "manage.py" in cmd:
        return "Django"
    if "code.py" in cmd:
        return "python3 (user code)"
    if "websockify" in cmd:
        return "websockify"
    return name


def find_container(override=None):
    try:
        out = subprocess.check_output(
            ["docker", "ps", "--format", "{{.ID}}\t{{.Names}}\t{{.Image}}"],
            text=True,
            stderr=subprocess.DEVNULL,
        )
        for line in out.strip().splitlines():
            parts = line.split("\t")
            if len(parts) < 3:
                continue
            cid, cname, cimage = parts
            if override and (override in cid or override in cname):
                return cid, cname, cimage
            if not override and any(img in cimage for img in RADI_IMAGES):
                return cid, cname, cimage
    except Exception:
        pass
    return None, None, None


def _parse_mb(s):
    s = s.strip()
    try:
        n = float(re.sub(r"[^\d.]", "", s))
        sl = s.lower()
        if "gib" in sl or "gb" in sl:
            return n * 1024
        if "mib" in sl or "mb" in sl:
            return n
        if "kib" in sl or "kb" in sl:
            return n / 1024
        return n / 1048576
    except Exception:
        return 0.0


def docker_stats(cid):
    try:
        out = subprocess.check_output(
            [
                "docker",
                "stats",
                "--no-stream",
                "--format",
                "{{.CPUPerc}}\t{{.MemUsage}}\t{{.NetIO}}\t{{.PIDs}}",
                cid,
            ],
            text=True,
            stderr=subprocess.DEVNULL,
            timeout=5,
        )
        p = out.strip().split("\t")
        mu, ml = [_parse_mb(x) for x in p[1].split(" / ")]
        nr, nt = [_parse_mb(x) for x in p[2].split(" / ")]
        return {
            "cpu_pct": float(p[0].replace("%", "")),
            "mem_mb": mu,
            "mem_limit_mb": ml,
            "net_rx_mb": nr,
            "net_tx_mb": nt,
            "pids": int(p[3]),
        }
    except Exception:
        return None


def container_processes(cid):
    try:
        r = subprocess.run(
            ["docker", "exec", cid, "python3", "-c", _PSUTIL_SCRIPT],
            capture_output=True,
            text=True,
            timeout=12,
        )
        if r.returncode == 0 and r.stdout.strip():
            return json.loads(r.stdout.strip())
    except Exception:
        pass
    return []


# Resolved once: "gz"/"ign" if present, "" if neither, None until probed. Caching by
# existence (not by a successful RTF read) avoids probing both binaries every sample
# while no simulation is running.
_rtf_binary_cache = None


def _detect_rtf_binary(cid):
    global _rtf_binary_cache
    if _rtf_binary_cache is not None:
        return _rtf_binary_cache
    for binary in ("gz", "ign"):
        try:
            r = subprocess.run(
                ["docker", "exec", cid, "bash", "-c", f"command -v {binary}"],
                capture_output=True,
                text=True,
                timeout=4,
            )
            if r.returncode == 0 and r.stdout.strip():
                _rtf_binary_cache = binary
                return binary
        except Exception:
            pass
    _rtf_binary_cache = ""
    return ""


def gazebo_rtf(cid):
    """Query Gazebo RTF via gz/ign topic /stats. Returns float or None."""
    binary = _detect_rtf_binary(cid)
    if not binary:
        return None
    try:
        r = subprocess.run(
            [
                "docker",
                "exec",
                cid,
                "bash",
                "-c",
                f"timeout 0.8 {binary} topic -e -t /stats 2>/dev/null | grep -m1 real_time_factor",
            ],
            capture_output=True,
            text=True,
            timeout=4,
        )
        m = re.search(r"real_time_factor:\s*([\d.]+)", r.stdout)
        if m:
            return round(float(m.group(1)), 3)
    except Exception:
        pass
    return None


def gpu_stats():
    try:
        out = subprocess.check_output(
            [
                "nvidia-smi",
                "--query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu",
                "--format=csv,noheader,nounits",
            ],
            text=True,
            stderr=subprocess.DEVNULL,
            timeout=3,
        )
        v = [x.strip() for x in out.strip().splitlines()[0].split(",")]
        return {
            "vendor": "nvidia",
            "util_pct": float(v[0]),
            "mem_used_mb": float(v[1]),
            "mem_total_mb": float(v[2]),
            "temp_c": float(v[3]),
        }
    except Exception:
        pass
    try:
        for card in sorted(os.listdir("/sys/class/drm")):
            bp = f"/sys/class/drm/{card}/device/gpu_busy_percent"
            if os.path.exists(bp):
                with open(bp) as f:
                    return {
                        "vendor": "intel_amd",
                        "util_pct": float(f.read().strip()),
                        "mem_used_mb": None,
                        "mem_total_mb": None,
                        "temp_c": None,
                    }
    except Exception:
        pass
    return None


def host_stats():
    vm = psutil.virtual_memory()
    return {
        "cpu_pct": psutil.cpu_percent(interval=None),
        "mem_used_gb": round(vm.used / 1024**3, 2),
        "mem_total_gb": round(vm.total / 1024**3, 1),
    }


def detect_exercise(processes):
    """Parse gz sim / gzserver cmdline to extract the active world name."""
    for p in processes:
        cmd, name = p.get("cmd", ""), p.get("name", "")
        is_sim = (
            "gzserver" in name
            or "gzserver" in cmd
            or ("gz sim" in cmd and ("-s " in cmd or "-r " in cmd))
        )
        if is_sim:
            m = re.search(r"[/\\]([\w_-]+)\.world", cmd)
            if m:
                return m.group(1)
    return None


def save_session(path, samples, meta):
    with open(path, "w") as f:
        json.dump({"metadata": meta, "samples": samples}, f, separators=(",", ":"))
