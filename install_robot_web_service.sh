#!/bin/sh
set -eu

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
SERVICE_NAME="robot-web-ui.service"
SERVICE_PATH="/etc/systemd/system/$SERVICE_NAME"
WEB_HOST="${WEB_HOST:-0.0.0.0}"
WEB_PORT="${WEB_PORT:-8088}"

cat <<EOF | sudo tee "$SERVICE_PATH" >/dev/null
[Unit]
Description=Robot Web Control UI
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$(id -un)
WorkingDirectory=$SCRIPT_DIR
ExecStart=/usr/bin/env python3 $SCRIPT_DIR/robot_web_ui.py --host $WEB_HOST --port $WEB_PORT
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now "$SERVICE_NAME"
echo "[SERVICE] Installed and started $SERVICE_NAME"
echo "[SERVICE] Open http://<RASPBERRY_PI_IP>:$WEB_PORT"
