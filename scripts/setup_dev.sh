#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="$ROOT_DIR/.venv"

echo "RoboticsAcademy developer setup"
echo "workspace: $ROOT_DIR"

python3 -V >/dev/null 2>&1 || { echo "python3 not found on PATH" >&2; exit 1; }

if [ -d "$VENV_DIR" ]; then
  echo "Using existing virtualenv at $VENV_DIR"
else
  echo "Creating virtualenv at $VENV_DIR"
  if python3 -m venv "$VENV_DIR" 2>/dev/null; then
    echo "Created virtualenv with python3 -m venv"
  else
    echo "python3 venv not available, trying to install virtualenv (user install) and retry"
    python3 -m pip install --user virtualenv
    python3 -m virtualenv "$VENV_DIR"
  fi
fi

echo "Activating virtualenv"
# shellcheck source=/dev/null
source "$VENV_DIR/bin/activate"

echo "Upgrading pip, setuptools and wheel"
pip install -U pip setuptools wheel

echo "Installing minimal dev/test requirements (tests/requirements.txt)"
pip install -r "$ROOT_DIR/tests/requirements.txt"

echo "Done. Next steps:"
echo "  source $VENV_DIR/bin/activate"
echo "  pytest -q"
echo
echo "If you want the full project runtime dependencies you can run (may require system packages):"
echo "  pip install -r $ROOT_DIR/docs/requirements.txt"
echo "Tip: If you need to install system packages on Debian/Ubuntu for venv support:"
echo "  sudo apt install python3-venv"

exit 0
