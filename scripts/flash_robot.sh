#!/bin/bash
# Прошивка ESP32-S3: metalinspector_robot (ездa + змейка + ToF, без камеры)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SKETCH="$ROOT/firmware/metalinspector_robot"
FQBN="${FQBN:-esp32:esp32:esp32s3}"

PORT="${1:-}"
if [ -z "$PORT" ]; then
  PORT="$(ls /dev/cu.wchusbserial* /dev/cu.usbmodem* /dev/cu.usbserial* 2>/dev/null | head -1 || true)"
fi
if [ -z "$PORT" ]; then
  echo "ESP32 не найден. Подключи data-USB и укажи порт:"
  echo "  bash scripts/flash_robot.sh /dev/cu.wchusbserialXXXX"
  arduino-cli board list 2>/dev/null || true
  exit 1
fi

echo "compile $FQBN …"
arduino-cli compile -b "$FQBN" "$SKETCH"
echo "upload → $PORT …"
arduino-cli upload -p "$PORT" -b "$FQBN" "$SKETCH"
echo "OK. Дальше: bash control/start_bridge.sh → http://127.0.0.1:8765/"
