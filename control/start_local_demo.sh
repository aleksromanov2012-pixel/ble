#!/bin/bash
# Завтрашнее демо: локальный сайт + USB-мост к ESP (LaunchAgents).
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
UID_NUM=$(id -u)
AGENTS="$ROOT/control/launchagents"
mkdir -p "$HOME/Library/LaunchAgents"
cp "$AGENTS/ru.metalinspector.local-server.plist" "$HOME/Library/LaunchAgents/"
cp "$AGENTS/ru.metalinspector.local-bridge.plist" "$HOME/Library/LaunchAgents/"

for label in ru.metalinspector.local-server ru.metalinspector.local-bridge; do
  launchctl bootout "gui/$UID_NUM/$label" 2>/dev/null || true
  launchctl bootstrap "gui/$UID_NUM" "$HOME/Library/LaunchAgents/$label.plist"
  launchctl kickstart -k "gui/$UID_NUM/$label" 2>/dev/null || true
done

for i in $(seq 1 30); do
  curl -sf -o /dev/null --max-time 1 http://127.0.0.1:8712/app/ && break
  sleep 0.5
done

echo "SITE   http://127.0.0.1:8712/app/"
echo -n "server "; curl -sf -o /dev/null http://127.0.0.1:8712/app/ && echo OK || echo DOWN
echo -n "bridge "; pgrep -f 'control/site_usb_bridge.py' >/dev/null && echo OK || echo DOWN
echo "ESP USB:"; ls /dev/cu.usbmodem* /dev/cu.wchusbserial* 2>/dev/null || echo "  (нет — воткни кабель)"
open "http://127.0.0.1:8712/app/" 2>/dev/null || true
