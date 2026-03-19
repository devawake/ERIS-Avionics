#!/bin/bash
# ERIS Avionics — Pi Zero 2W Setup Script
# Run ONCE on the avionics Pi to install the systemd autostart service.
#
# Usage:  sudo bash setup_avionics_pi.sh

set -e
cd "$(dirname "$0")"
CURRENT_DIR="$(pwd)"
USER_NAME="${SUDO_USER:-$(whoami)}"

echo "=========================================="
echo "  ERIS Avionics Pi Setup"
echo "  Installing systemd autostart service"
echo "=========================================="

# ── Dependencies ────────────────────────────────────────────────────────────
echo "Installing/verifying dependencies..."
apt-get update -qq
apt-get install -y git python3-pip python3-venv

# ── Python venv ─────────────────────────────────────────────────────────────
if [ ! -d "$CURRENT_DIR/venv" ]; then
    echo "Creating Python venv..."
    python3 -m venv "$CURRENT_DIR/venv"
fi

if [ -f "$CURRENT_DIR/requirements.txt" ]; then
    echo "Installing Python requirements..."
    "$CURRENT_DIR/venv/bin/pip" install -r "$CURRENT_DIR/requirements.txt" --quiet
fi

# ── Recordings directory ─────────────────────────────────────────────────────
echo "Creating recordings directory..."
mkdir -p /home/eris/ERIS-Avionics/recordings
chown -R "$USER_NAME":"$USER_NAME" /home/eris/ERIS-Avionics || true

# ── Make scripts executable ──────────────────────────────────────────────────
chmod +x "$CURRENT_DIR/autostart_avionics.sh"

# ── Systemd service ──────────────────────────────────────────────────────────
echo "Writing systemd service file..."
cat > /etc/systemd/system/eris-avionics.service << EOF
[Unit]
Description=ERIS Avionics Flight Computer
# Start after basic system is up; does NOT require network (pad will be offline)
After=multi-user.target
Wants=multi-user.target

[Service]
Type=simple
User=root
WorkingDirectory=$CURRENT_DIR
ExecStart=$CURRENT_DIR/autostart_avionics.sh
Restart=on-failure
RestartSec=10
StandardOutput=journal
StandardError=journal
# Ensure GPIO/SPI access
SupplementaryGroups=gpio spi i2c dialout video

[Install]
WantedBy=multi-user.target
EOF

echo "Enabling systemd service..."
systemctl daemon-reload
systemctl enable eris-avionics.service

echo ""
echo "=========================================="
echo "  Setup Complete!"
echo ""
echo "  The Pi Zero will now:"
echo "    1. Boot into the flight computer automatically"
echo "    2. Wait up to 10 s for Wi-Fi"
echo "    3. Run 'git pull' if connected"
echo "    4. Launch main.py (GPIO 16 = ARM switch)"
echo ""
echo "  Test manually: sudo systemctl start eris-avionics.service"
echo "  View logs:     journalctl -u eris-avionics.service -f"
echo "  Stop service:  sudo systemctl stop eris-avionics.service"
echo "=========================================="
