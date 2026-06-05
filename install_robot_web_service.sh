#!/bin/sh
set -eu

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
SERVICE_NAME="robot-web-ui.service"
SERVICE_PATH="/etc/systemd/system/$SERVICE_NAME"
ENV_FILE="$SCRIPT_DIR/.robot_web_ui.env"
WEB_HOST="${WEB_HOST:-0.0.0.0}"
WEB_PORT="${WEB_PORT:-8088}"
VENV_PATH="${VENV_PATH:-}"
VK_ENABLED="${VK_ENABLED:-1}"
VK_ACCESS_TOKEN="${VK_ACCESS_TOKEN:-}"
VK_PEER_ID="${VK_PEER_ID:-}"
VK_API_VERSION="${VK_API_VERSION:-5.199}"

chmod +x "$SCRIPT_DIR/run_robot_web.sh"

cat > "$ENV_FILE" <<EOF
HOST="$WEB_HOST"
PORT="$WEB_PORT"
VENV_PATH="$VENV_PATH"
VK_ENABLED="$VK_ENABLED"
VK_ACCESS_TOKEN="$VK_ACCESS_TOKEN"
VK_PEER_ID="$VK_PEER_ID"
VK_API_VERSION="$VK_API_VERSION"
EOF

chmod 600 "$ENV_FILE"

cat <<EOF | sudo tee "$SERVICE_PATH" >/dev/null
[Unit]
Description=Robot Web Control UI
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$(id -un)
WorkingDirectory=$SCRIPT_DIR
EnvironmentFile=$ENV_FILE
ExecStart=$SCRIPT_DIR/run_robot_web.sh
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now "$SERVICE_NAME"
echo "[SERVICE] Installed and started $SERVICE_NAME"
echo "[SERVICE] Open http://<RASPBERRY_PI_IP>:$WEB_PORT"
echo "[SERVICE] VK notifications are controlled through $ENV_FILE"
