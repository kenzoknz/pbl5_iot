#!/usr/bin/env bash
set -euo pipefail

echo "[JETSON-UART] Configure /dev/ttyTHS1 for ESP32 communication"

if ! command -v systemctl >/dev/null 2>&1; then
  echo "[JETSON-UART] systemctl not found. Run these steps manually on Jetson Linux."
  exit 1
fi

if systemctl list-unit-files | grep -q '^nvgetty\.service'; then
  echo "[JETSON-UART] Stopping nvgetty service..."
  sudo systemctl stop nvgetty || true
  sudo systemctl disable nvgetty || true
fi

echo "[JETSON-UART] Ensure current user is in dialout group..."
sudo usermod -a -G dialout "$USER"

echo "[JETSON-UART] Create udev rule for ttyTHS1 permissions..."
sudo tee /etc/udev/rules.d/99-jetson-uart.rules >/dev/null <<'RULE'
KERNEL=="ttyTHS1", MODE="0666", GROUP="dialout"
RULE

sudo udevadm control --reload-rules
sudo udevadm trigger

if [[ -e /dev/ttyTHS1 ]]; then
  echo "[JETSON-UART] UART device detected: /dev/ttyTHS1"
  ls -l /dev/ttyTHS1
else
  echo "[JETSON-UART] WARNING: /dev/ttyTHS1 not found. Check Jetson UART pinmux settings."
fi

echo "[JETSON-UART] Done. Re-login (or reboot) to apply dialout group changes."
