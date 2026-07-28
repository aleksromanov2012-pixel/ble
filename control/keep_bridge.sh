#!/bin/bash
# Watchdog без pkill -f: только start_bridge.sh по PID
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LOG=/tmp/ble-bridge-watch.log
LOCK=/tmp/ble-bridge-watch.lock
export ROBOT_LINK="${ROBOT_LINK:-ble}"

if ! mkdir "$LOCK" 2>/dev/null; then
  echo "watchdog уже запущен"
  exit 0
fi
# do not trap-rm lock on exit if we daemonize below

alive() {
  curl -sS -m 1 -o /dev/null http://127.0.0.1:8765/api/status 2>/dev/null
}

(
  echo "[$(date '+%F %T')] watchdog start" >>"$LOG"
  while true; do
    if ! alive; then
      echo "[$(date '+%H:%M:%S')] down → start_bridge" >>"$LOG"
      bash "$ROOT/control/start_bridge.sh" >>"$LOG" 2>&1 || true
    fi
    sleep 5
  done
) >>"$LOG" 2>&1 &
echo "watchdog pid $!"
