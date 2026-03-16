Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $root

Write-Host "[0/8] Waiting for Docker engine..."
for ($i = 1; $i -le 90; $i++) {
    docker info *> $null
    if ($LASTEXITCODE -eq 0) { break }
    Start-Sleep -Seconds 2
}
docker info *> $null
if ($LASTEXITCODE -ne 0) {
    throw "Docker engine is not ready. Start Docker Desktop first."
}

Write-Host "[1/8] Restarting stack cleanly..."
docker compose down -v --remove-orphans
docker compose up -d

Write-Host "[2/8] Waiting for database healthy..."
for ($i = 1; $i -le 90; $i++) {
    $h = docker inspect --format "{{.State.Health.Status}}" my-postgres 2>$null
    if ($h -eq "healthy") { break }
    Start-Sleep -Seconds 2
}

Write-Host "[3/8] Forcing PostgreSQL Django settings..."
$dbPatchPy = @'
from pathlib import Path
p = Path('/RoboticsAcademy/academy/settings.py')
text = p.read_text(encoding='utf-8')
block = """
DATABASES = {
    "default": {
        "ENGINE": "django.db.backends.postgresql",
        "NAME": "academy_db",
        "USER": "user-dev",
        "PASSWORD": "robotics-academy-dev",
        "HOST": "my-postgres",
        "PORT": "5432",
    }
}
ALLOWED_HOSTS = ["*"]
"""
if "DATABASES = {" in text:
    text += "\n\n" + block + "\n"
else:
    text += "\n" + block + "\n"
p.write_text(text, encoding='utf-8')
print("postgres db patch applied")
'@
$dbPatchPy | docker exec -i developer-container python3 -

Write-Host "[4/8] Running migrate..."
docker exec developer-container bash -lc "cd /RoboticsAcademy && python3 manage.py migrate --noinput"

Write-Host "[5/8] Patching backend tags parser..."
Get-Content -Raw "$root\patch_backend_tags_parser.py" | docker exec -i developer-container python3 -
docker exec developer-container python3 -m py_compile /RoboticsAcademy/academy/views.py | Out-Null

Write-Host "[6/8] Fixing follow_line metadata..."
$sql = @'
UPDATE exercises
SET tags = $$["ROS2","AUTONOMOUS DRIVING"]$$
WHERE exercise_id='follow_line';

DELETE FROM exercises_tools
WHERE exercise_id=(SELECT id FROM exercises WHERE exercise_id='follow_line')
  AND tool_id='console';
'@
$sql | docker exec -i my-postgres psql -U user-dev -d academy_db

Write-Host "[6.5/8] Applying follow_line baseline code..."
Get-Content -Raw "$root\patch_follow_line_baseline.py" | docker exec -i developer-container python3 -
docker exec developer-container bash -lc "pylint /RoboticsAcademy/filesystem/follow_line/academy.py -sn >/dev/null"

Write-Host "[7/8] Patching RAM console fd crash + starting services..."
Get-Content -Raw "$root\patch_ram_console_fd.py" | docker exec -i developer-container python3 -

# Workaround for RAM versions that still call max(map(int, fds[:-1])) on /dev/pts.
# Keep one numeric PTY alive so run_application never sees an empty sequence.
docker exec developer-container bash -lc "pkill -f 'python3 -c import pty,time; pty.openpty(); time.sleep(10**6)' >/dev/null 2>&1 || true"
docker exec -d developer-container python3 -c "import pty,time; pty.openpty(); time.sleep(10**6)"

$vncWatchdogPy = @'
from pathlib import Path
script = "\n".join([
    "#!/bin/bash",
    "while true; do",
    '  if pgrep -f "Xorg .*:0" >/dev/null 2>&1; then',
    '    if ! ss -lnt 2>/dev/null | grep -q "127.0.0.1:5900"; then',
    "      x11vnc -display :0 -rfbport 5900 -localhost -forever -shared -bg >/tmp/x11vnc-autofix.log 2>&1",
    "    fi",
    "  fi",
    "  sleep 2",
    "done",
    "",
])
Path('/tmp/vnc-watchdog.sh').write_text(script, encoding='utf-8')
'@
$vncWatchdogPy | docker exec -i developer-container python3 -
docker exec developer-container chmod +x /tmp/vnc-watchdog.sh | Out-Null
docker exec developer-container bash -lc "pkill -f '/tmp/vnc-watchdog.sh' >/dev/null 2>&1 || true" | Out-Null
docker exec -d developer-container /tmp/vnc-watchdog.sh | Out-Null

docker exec developer-container bash -lc "pkill -f 'manage.py runserver' >/dev/null 2>&1 || true"
docker exec developer-container bash -lc "pkill -f 'robotics_application_manager.*manager.py' >/dev/null 2>&1 || true"
docker exec -d developer-container bash -lc "cd /RoboticsAcademy && python3 manage.py runserver 0.0.0.0:7164 --noreload"
docker exec -d developer-container bash -lc "python3 /usr/local/lib/python3.10/dist-packages/robotics_application_manager/manager.py"

Write-Host "[8/8] Health check..."
Start-Sleep -Seconds 4
$exerciseList = curl.exe -s "http://127.0.0.1:7164/academy/get_exercise_list/"
if (-not $exerciseList) {
    throw "Backend endpoint did not respond."
}

Write-Host ""
Write-Host "Ready: http://127.0.0.1:7164/academy/"
