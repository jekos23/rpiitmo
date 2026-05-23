#!/bin/bash

echo "=========================================="
echo "Установка удаленного доступа Tailscale"
echo "=========================================="

echo "[1/3] Установка Tailscale..."
curl -fsSL https://tailscale.com/install.sh | sh

echo "[2/3] Активация Tailscale и SSH..."
echo "Сейчас появится ссылка. Откройте ее на вашем телефоне или ПК,"
echo "чтобы авторизовать робота в вашей сети."
sudo tailscale up --ssh --hostname=tlabitmopy

echo "[3/3] Настройка завершена!"
echo "IP-адрес робота в сети Tailscale:"
tailscale ip -4
echo "Также вы всегда можете увидеть его IP в приложении Tailscale на вашем телефоне."
echo "Теперь вы можете безопасно подключаться к роботу по SSH из любой точки мира!"
