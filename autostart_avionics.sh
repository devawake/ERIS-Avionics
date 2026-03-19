#!/bin/bash
# ERIS Avionics — Autostart Script
# Mirrors the groundstation autostart.sh pattern.
# Runs on boot via systemd (eris-avionics.service).
#
# Flow:
#   1. Wait up to 10 s for Wi-Fi (short — won't be online on the pad)
#   2. git pull if connected
#   3. Run main.py with root (required for GPIO / SPI)

set -e
cd "$(dirname "$0")"
SCRIPT_DIR="$(pwd)"

echo "=========================================="
echo "  ERIS Avionics Autostart"
echo "  $(date)"
echo "=========================================="

# ── 1. Try Wi-Fi / git pull ────────────────────────────────────────────────
echo "Checking for network (up to 10 s)..."
CONNECTED=0
for i in {1..10}; do
    if ping -q -c 1 -W 1 8.8.8.8 > /dev/null 2>&1; then
        CONNECTED=1
        echo "Network up (attempt $i). Pulling latest code..."
        git pull origin main && echo "git pull OK" || echo "git pull FAILED (continuing)"
        break
    fi
    sleep 1
done

if [ "$CONNECTED" -eq 0 ]; then
    echo "No network within 10 s — skipping git pull (expected on pad)."
fi

# ── 2. Launch flight computer ───────────────────────────────────────────────
echo "Starting ERIS flight computer..."

# Use the venv if it exists, otherwise fall back to system python3
if [ -f "$SCRIPT_DIR/venv/bin/python3" ]; then
    PYTHON="$SCRIPT_DIR/venv/bin/python3"
else
    PYTHON="python3"
fi

# GPIO and SPI require root
exec sudo "$PYTHON" "$SCRIPT_DIR/main.py"
