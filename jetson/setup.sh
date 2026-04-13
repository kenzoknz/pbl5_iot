#!/usr/bin/env bash
set -euo pipefail

echo "[JETSON-SETUP] Configure UART /dev/ttyTHS1 for ESP32 bridge"

echo "[1/5] Disable nvgetty"
sudo systemctl stop nvgetty || true
sudo systemctl disable nvgetty || true
sudo udevadm trigger

echo "[2/5] Verify UART device"
if [[ -e /dev/ttyTHS1 ]]; then
  ls -l /dev/ttyTHS1
else
  echo "[WARN] /dev/ttyTHS1 not found. Check Jetson pinmux/UART config first."
fi

echo "[3/5] Add current user to dialout and set UART permissions"
sudo usermod -a -G dialout "$USER"
sudo chmod 666 /dev/ttyTHS1 || true

echo "[4/5] Test UART connection"
python3 -c "import serial; s=serial.Serial('/dev/ttyTHS1', 115200, timeout=0.1); print('Connected'); s.close()"

echo "[5/5] Install Python dependencies"
python3 -m pip install --upgrade pip
python3 -m pip install pyserial opencv-python-headless ultralytics

echo "[JETSON-SETUP] Done. If group changes do not apply, re-login or reboot."